/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2026 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MELoop.cpp
/// @author  Daniel Krajzewicz
/// @date    Tue, May 2005
///
// The main mesocopic simulation loop
/****************************************************************************/
#include <config.h>

#include <queue>
#include <vector>
#include <algorithm>
#include <map>
#include <cmath>
#include <limits>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSJunction.h>
#include <utils/common/MsgHandler.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>
#include <utils/common/ToString.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/RandHelper.h>
#include "MELoop.h"
#include "MESegment.h"
#include "MELSegment.h"
#include "MEVehicle.h"

long long int MELoop::LeaderEvent::myEventCounter(0);


// ===========================================================================
// MELoop::WorkerPool: runs a task for the indices 0..n-1 on all threads and waits
// ===========================================================================
class MELoop::WorkerPool {
public:
    WorkerPool(int numThreads) {
        for (int i = 1; i < numThreads; i++) {
            myThreads.emplace_back([this] { work(); });
        }
    }

    ~WorkerPool() {
        {
            std::lock_guard<std::mutex> lock(myMutex);
            myStop = true;
            myGeneration++;
        }
        myWake.notify_all();
        for (std::thread& t : myThreads) {
            t.join();
        }
    }

    void run(int count, const std::function<void(int)>& task) {
        {
            std::lock_guard<std::mutex> lock(myMutex);
            myTask = &task;
            myCount = count;
            myNext = 0;
            myActive = (int)myThreads.size();
            myGeneration++;
        }
        myWake.notify_all();
        execute();
        std::unique_lock<std::mutex> lock(myMutex);
        myDone.wait(lock, [this] { return myActive == 0; });
    }

private:
    void execute() {
        for (int i = myNext.fetch_add(1); i < myCount; i = myNext.fetch_add(1)) {
            (*myTask)(i);
        }
    }

    void work() {
        unsigned long long seen = 0;
        while (true) {
            {
                std::unique_lock<std::mutex> lock(myMutex);
                myWake.wait(lock, [this, seen] { return myGeneration != seen; });
                seen = myGeneration;
                if (myStop) {
                    return;
                }
            }
            execute();
            std::lock_guard<std::mutex> lock(myMutex);
            if (--myActive == 0) {
                myDone.notify_one();
            }
        }
    }

    std::vector<std::thread> myThreads;
    std::mutex myMutex;
    std::condition_variable myWake;
    std::condition_variable myDone;
    std::atomic<int> myNext{0};
    int myCount = 0;
    int myActive = 0;
    const std::function<void(int)>* myTask = nullptr;
    unsigned long long myGeneration = 0;
    bool myStop = false;
};

// ===========================================================================
// method definitions
// ===========================================================================
MELoop::MELoop(const SUMOTime recheckInterval) : myFullRecheckInterval(recheckInterval), myLinkRecheckInterval(TIME2STEPS(1)) {
}

MELoop::~MELoop() {
    if (myThreads > 0 && myVerbose) {
        long long int events = 0;
        double busy = 0;
        for (const Group& g : myGroups) {
            events += g.events;
            busy += g.busy;
        }
        std::cout << "Parallel meso: " << myRoundCount << " rounds, " << events << " events, " << myDeferredCount
                  << " deferred, parallel part " << myParallelTime << "s (group work " << busy << "s), serial part " << mySerialTime << "s\n";
    }
    delete myWorkerPool;
    for (Group& g : myGroups) {
        delete g.rng;
    }
    for (std::vector<MESegment*>::const_iterator j = myEdges2FirstSegments.begin(); j != myEdges2FirstSegments.end(); ++j) {
        for (MESegment* s = *j; s != nullptr;) {
            MESegment* n = s->getNextSegment();
            delete s;
            s = n;
        }
    }
}


void
MELoop::simulate(SUMOTime tMax) {
    if (!myGroupsInitialized) {
        initGroups();
    }
    myStepEnd = tMax;
    if (myThreads <= 0) {
        processEvents(myLeaderCars, myInvalidatedLeaderCars, tMax, -1);
        return;
    }
    const std::function<void(int)> task = [this, tMax](int i) {
        const auto start = std::chrono::steady_clock::now();
        processEvents(myGroups[i].leaders, myGroups[i].invalidated, tMax, i);
        myGroups[i].busy += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    };
    for (int round = 0; round < myRounds; round++) {
        bool due = false;
        for (const Group& g : myGroups) {
            if (!g.leaders.empty() && g.leaders.top().time <= tMax) {
                due = true;
                break;
            }
        }
        if (!due) {
            break;
        }
        const auto t0 = std::chrono::steady_clock::now();
        myWorkerPool->run((int)myGroups.size(), task);
        const auto t1 = std::chrono::steady_clock::now();
        myParallelTime += std::chrono::duration<double>(t1 - t0).count();
        myRoundCount++;
        // serial part: moves into other groups and teleports, in time order
        std::vector<MEVehicle*> deferred;
        for (Group& g : myGroups) {
            deferred.insert(deferred.end(), g.deferred.begin(), g.deferred.end());
            g.deferred.clear();
        }
        std::sort(deferred.begin(), deferred.end(), [](const MEVehicle * a, const MEVehicle * b) {
            return a->getEventTime() < b->getEventTime() || (a->getEventTime() == b->getEventTime() && a->getNumericalID() < b->getNumericalID());
        });
        for (MEVehicle* veh : deferred) {
            veh->setDeferred(false);
            // the vehicle may have been overtaken in the meantime and then waits for its next turn as leader
            MESegment* const seg = veh->getSegment();
            if (seg != nullptr && veh->getQueIndex() != MESegment::PARKING_QUEUE
                    && (seg->getQueue(veh->getQueIndex()).empty() || seg->getQueue(veh->getQueIndex()).back() != veh)) {
                continue;
            }
            checkCar(veh, -1);
        }
        myDeferredCount += (long long int)deferred.size();
        mySerialTime += std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
    }
}


void
MELoop::processEvents(LeaderEventQeue& leaders, LeaderEventQeue& invalidated, SUMOTime tMax, const int group) {
    while (!leaders.empty()) {
        LeaderEvent e = leaders.top();
        assert(group >= 0 || e.time > tMax - DELTA_T);
        if (e.time > tMax) {
            return;
        }
        leaders.pop();
        while (!invalidated.empty() && e.time > invalidated.top().time) {
            invalidated.pop();
        }
        if (!invalidated.empty()) {
            if (wasInvalidated(e, invalidated)) {
                continue;
            }
        }
        //std::cout << SIMTIME << " checkChar " << e.veh->getID() << " on=" << e.veh->getSegment()->getID() << " t=" << e.time << " nid=" << e.veh->getNumericalID() << "\n";
        if (group >= 0) {
            myGroups[group].events++;
        }
        checkCar(e.veh, group);
        assert(leaders.empty() || leaders.top().time >= e.time);
    }
}


bool
MELoop::wasInvalidated(const LeaderEvent& e, LeaderEventQeue& invalidated) {
    if (e.time == invalidated.top().time) {
        // we need to check whether any of the events with the same time
        // is our current vehicle because the order in which
        // removeLeaderCar was called is unrelated to the order of addLeaderCar
        std::vector<LeaderEvent> tmp;
        bool curInvalid = false;
        while (!invalidated.empty() && e.time == invalidated.top().time) {
            if (e == invalidated.top()) {
                invalidated.pop();
                curInvalid = true;
                break;
            } else {
                tmp.push_back(invalidated.top());
                invalidated.pop();
            }
        }
        for (LeaderEvent& le : tmp) {
            invalidated.push(le);
        }
        return curInvalid;
    }
    return false;
}


SUMOTime
MELoop::changeSegment(MEVehicle* veh, SUMOTime leaveTime, MESegment* const toSegment, MSMoveReminder::Notification reason, const bool ignoreLink) const {
    int qIdx = 0;
    MESegment* const onSegment = veh->getSegment();
    if (MESegment::isInvalid(toSegment)) {
        if (veh->isStoppedTriggered()) {
            return leaveTime + MAX2(SUMOTime(1), myLinkRecheckInterval);
        }
        if (onSegment != nullptr) {
            onSegment->send(veh, toSegment, qIdx, leaveTime, reason);
        } else {
            WRITE_WARNINGF(TL("Vehicle '%' teleports beyond arrival edge '%', time=%."),
                           veh->getID(), veh->getEdge()->getID(), time2string(leaveTime));
        }
        veh->setSegment(toSegment); // signal arrival
        MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(veh);
        return leaveTime;
    } else if ((!MSGlobals::gCheckRoutes || veh->ignoreTransientPermissions())
               && !ignoreLink
               && !MESegment::isInvalid(onSegment)
               && &onSegment->getEdge() != &toSegment->getEdge()
               && veh->getEdge()->allowedLanes(*veh->succEdge(1), veh->getVClass()) == nullptr) {
        if (veh->isStopped()) {
            veh->processStop();
        }
        // all queues on the next segment are forbidden. We don't want to check
        // too often whether the forbiden edge has become permitted again
        const SUMOTime ttt = veh->getVehicleType().getParameter().getTimeToTeleport(MSGlobals::gTimeToGridlock);
        if (ttt > 0) {
            // if teleporting is enabled, make sure we look at the vehicle when the gridlock-time is up
            const SUMOTime recheck = MSGlobals::gTimeToTeleportDisconnected >= 0 ? MIN2(ttt, MSGlobals::gTimeToTeleportDisconnected) : ttt;
            return MAX2(MIN2(leaveTime + myLinkRecheckInterval, MIN2(veh->getBlockTime(), leaveTime) + recheck + 1), leaveTime + 1);
        }
        return leaveTime + myLinkRecheckInterval;
    }
    toSegment->updateEntryBlockTime(leaveTime);
    const SUMOTime entry = toSegment->hasSpaceFor(veh, leaveTime, qIdx);
    if (entry == leaveTime && (ignoreLink || veh->mayProceed())) {
        if (onSegment != nullptr) {
            if (veh->getQueIndex() == MESegment::PARKING_QUEUE) { // parking or just aborted parking
                if (veh->isParking()) {
                    veh->processStop();
                }
                veh->getEdge()->getLanes()[0]->removeParking(veh);  // TODO for GUI only
            } else {
                onSegment->send(veh, toSegment, qIdx, leaveTime, onSegment->getNextSegment() == nullptr ? MSMoveReminder::NOTIFICATION_JUNCTION : MSMoveReminder::NOTIFICATION_SEGMENT);
            }
            toSegment->receive(veh, qIdx, leaveTime, false, ignoreLink, &onSegment->getEdge() != &toSegment->getEdge());
        } else {
            WRITE_WARNINGF(TL("Vehicle '%' ends teleporting on edge '%':%, time=%."),
                           veh->getID(), toSegment->getEdge().getID(), toSegment->getIndex(), time2string(leaveTime));
            // this is not quite correct but suffices for interrogation by
            // subsequent methods (veh->getSpeed() needs segment != 0)
            veh->setSegment(myEdges2FirstSegments[veh->getEdge()->getNumericalID()]);
            // clean up detectors (do not add traffic data)
            // note: updateDatector is not called if leaveTime == getLastEntryTime()
            veh->updateDetectors(veh->getLastEntryTime(), veh->getEventTime(), true, MSMoveReminder::NOTIFICATION_TELEPORT);
            toSegment->receive(veh, qIdx, leaveTime, false, true, true);
        }
        return entry;
    } else {
        if (veh->getQueIndex() != MESegment::PARKING_QUEUE && veh->isStopped() && veh->endTriggeredStop()) {
            veh->processStop();
        }
    }
    if (entry == leaveTime && !ignoreLink) { // this is a long way of saying !veh->mayProceed() (which is a costly call)
        return entry + MAX2(SUMOTime(1), myLinkRecheckInterval);
    }
    return entry;
}


void
MELoop::checkCar(MEVehicle* veh, const int group) {
    const SUMOTime leaveTime = veh->getEventTime();
    MESegment* const onSegment = veh->getSegment();
    MESegment* const toSegment = veh->getQueIndex() == MESegment::PARKING_QUEUE ? onSegment : nextSegment(onSegment, veh);
    const bool teleporting = (onSegment == nullptr); // is the vehicle currently teleporting?
    if (group >= 0 && (teleporting || (!MESegment::isInvalid(toSegment) && toSegment->getGroup() != group))) {
        // entering another group's segment is decided in the serial part of the step
        veh->setDeferred(true);
        myGroups[group].deferred.push_back(veh);
        return;
    }
    // @note reason is only evaluated if toSegment == nullptr
    const SUMOTime nextEntry = changeSegment(veh, leaveTime, toSegment, MSMoveReminder::NOTIFICATION_ARRIVED, teleporting);
    if (nextEntry == leaveTime) {
        return;
    }
    const SUMOTime ttt = veh->getVehicleType().getParameter().getTimeToTeleport(MSGlobals::gTimeToGridlock);
    const bool r1 = ttt > 0 && veh->getWaitingTime() > ttt;
    const bool r3 = MSGlobals::gTimeToTeleportDisconnected >= 0 && veh->getWaitingTime() > MSGlobals::gTimeToTeleportDisconnected;
    if (!veh->isStopped() && (r1 || r3)) {
        const bool disconnected = (MSGlobals::gTimeToTeleportDisconnected >= 0
                                   && veh->succEdge(1) != nullptr
                                   && veh->getEdge()->allowedLanes(*veh->succEdge(1), veh->getVClass()) == nullptr);
        if ((r1 && !disconnected) || (r3 && disconnected)) {
            if (group >= 0) {
                veh->setDeferred(true);
                myGroups[group].deferred.push_back(veh);
                return;
            }
            teleportVehicle(veh, toSegment, disconnected);
            return;
        }
    }
    if (veh->getBlockTime() == SUMOTime_MAX && (!veh->isStopped()
            || (!veh->isStoppedTriggered() && veh->isStoppedParking()))) {
        veh->setBlockTime(leaveTime);
    }
    if (nextEntry == SUMOTime_MAX) {
        // all usable queues on the next segment are full
        SUMOTime newEventTime = MAX3(toSegment->getEventTime() + 1, leaveTime + 1, leaveTime + myFullRecheckInterval);
        if (group >= 0 && toSegment->getEventTime() < leaveTime) {
            // the vehicle blocking the next segment waits for the serial part of this step
            newEventTime = MAX2(newEventTime, myStepEnd + 1);
        }
        if (ttt) {
            // if teleporting is enabled, make sure we look at the vehicle when the gridlock-time is up
            const SUMOTime recheck = veh->getBlockTime() + 1 + ttt;
            if (recheck > leaveTime) {
                newEventTime = MIN2(newEventTime, recheck);
            }
        }
        if (MSGlobals::gTimeToTeleportDisconnected >= 0) {
            const SUMOTime recheck = veh->getBlockTime() + 1 + MSGlobals::gTimeToTeleportDisconnected;
            if (recheck > leaveTime && veh->getEdge()->allowedLanes(*veh->succEdge(1), veh->getVClass()) == nullptr) {
                newEventTime = MIN2(newEventTime, recheck);
            }
        }
        veh->setEventTime(newEventTime);
    } else {
        // receiving segment has recently received another vehicle or the junction is blocked
        veh->setEventTime(nextEntry);
    }
    addLeaderCar(veh, teleporting ? nullptr : onSegment->getLink(veh));
}


void
MELoop::teleportVehicle(MEVehicle* veh, MESegment* const toSegment, bool disconnected) {
    const SUMOTime leaveTime = veh->getEventTime();
    MESegment* const onSegment = veh->getSegment();
    if (MSGlobals::gRemoveGridlocked) {
        WRITE_WARNINGF(TL("Teleporting vehicle '%'; waited too long, from edge '%':%, time=%."),
                       veh->getID(), onSegment->getEdge().getID(), onSegment->getIndex(),
                       time2string(leaveTime));
        MSNet::getInstance()->getVehicleControl().registerTeleportJam();
        int qIdx = 0;
        onSegment->send(veh, nullptr, qIdx, leaveTime, MSMoveReminder::NOTIFICATION_TELEPORT_ARRIVED);
        veh->setSegment(nullptr);
        MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(veh);
        return;
    }
    const bool teleporting = (onSegment == nullptr); // is the vehicle already teleporting?
    // try to find a place on the current edge
    MESegment* teleSegment = disconnected ? toSegment : toSegment->getNextSegment();
    while (teleSegment != nullptr && changeSegment(veh, leaveTime, teleSegment, MSMoveReminder::NOTIFICATION_TELEPORT, true) != leaveTime) {
        // @caution the time to get to the next segment here is ignored XXX
        teleSegment = teleSegment->getNextSegment();
    }
    if (teleSegment != nullptr) {
        if (!teleporting) {
            // we managed to teleport in a single jump
            const std::string reason = disconnected ? " (disconnected)" : "";
            WRITE_WARNINGF(TL("Teleporting vehicle '%'; waited too long%, from edge '%':% to edge '%':%, time=%."),
                           veh->getID(), reason, onSegment->getEdge().getID(), onSegment->getIndex(),
                           teleSegment->getEdge().getID(), teleSegment->getIndex(), time2string(leaveTime));
            MSNet::getInstance()->getVehicleControl().registerTeleportJam();
        }
    } else {
        // teleport across the current edge and try insertion later
        if (!teleporting) {
            int qIdx = 0;
            // announce start of multi-step teleport, arrival will be announced in changeSegment()
            WRITE_WARNINGF(TL("Teleporting vehicle '%'; waited too long, from edge '%':%, time=%."),
                           veh->getID(), onSegment->getEdge().getID(), onSegment->getIndex(), time2string(leaveTime));
            MSNet::getInstance()->getVehicleControl().registerTeleportJam();
            // remove from current segment
            onSegment->send(veh, nullptr, qIdx, leaveTime, MSMoveReminder::NOTIFICATION_TELEPORT);
            // mark veh as teleporting
            veh->setSegment(nullptr);
        } else {
            veh->updateDetectors(veh->getLastEntryTime(), leaveTime, true, MSMoveReminder::NOTIFICATION_TELEPORT);
        }
        // @caution microsim uses current travel time teleport duration
        const SUMOTime teleArrival = leaveTime + TIME2STEPS(veh->getEdge()->getLength() / MAX2(veh->getEdge()->getSpeedLimit(), NUMERICAL_EPS));
        const bool atDest = veh->moveRoutePointer();
        if (atDest) {
            // teleporting to end of route
            changeSegment(veh, teleArrival, nullptr, MSMoveReminder::NOTIFICATION_TELEPORT_ARRIVED, true);
        } else {
            veh->setEventTime(teleArrival);
            addLeaderCar(veh, nullptr);
            // teleporting vehicles must react to rerouters
            getSegmentForEdge(*veh->getEdge())->addReminders(veh);
            veh->activateReminders(MSMoveReminder::NOTIFICATION_JUNCTION);
        }
    }
}


void
MELoop::addLeaderCar(MEVehicle* veh, MSLink* link) {
    if (!myGroupsInitialized) {
        initGroups();
    }
    const long long int loaded = veh->getLoadedEventIndex();
    veh->setLoadedEventIndex(-1);
    if (myThreads > 0) {
        Group& g = myGroups[getGroup(veh)];
        if (loaded >= 0) {
            // restore the order of simultaneous events from the loaded state
            g.leaders.push(LeaderEvent(veh, loaded));
            g.eventCounter = MAX2(g.eventCounter, loaded + 1);
        } else {
            g.leaders.push(LeaderEvent(veh, g.eventCounter++));
        }
    } else {
        LeaderEvent e(veh);
        if (loaded >= 0) {
            // restore the order of simultaneous events from the loaded state
            e.eventIndex = loaded;
            LeaderEvent::myEventCounter = MAX2(LeaderEvent::myEventCounter, e.eventIndex + 1);
        }
        myLeaderCars.push(e);
    }
    veh->setApproaching(link);
}


void
MELoop::clearState() {
    myLeaderCars = LeaderEventQeue();
    myInvalidatedLeaderCars = LeaderEventQeue();
    for (Group& g : myGroups) {
        g.leaders = LeaderEventQeue();
        g.invalidated = LeaderEventQeue();
        g.deferred.clear();
    }
}


long long int
MELoop::getEventIndex(const MEVehicle* veh) const {
    if (myEventIndexCacheTime != SIMSTEP) {
        myEventIndexCache.clear();
        // the ranks refer to the event queue holding the event (the serial one or the one of a parallel group)
        auto rank = [this](const LeaderEventQeue & leaders, const LeaderEventQeue & invalidated) {
            std::map<std::pair<SUMOTime, long long int>, int> invalid;
            for (LeaderEventQeue inv = invalidated; !inv.empty(); inv.pop()) {
                invalid[std::make_pair(inv.top().time, inv.top().nid)]++;
            }
            std::vector<std::pair<long long int, long long int> > pending;
            for (LeaderEventQeue q = leaders; !q.empty(); q.pop()) {
                const LeaderEvent& e = q.top();
                auto it = invalid.find(std::make_pair(e.time, e.nid));
                if (it != invalid.end() && it->second > 0) {
                    it->second--;
                    continue;
                }
                pending.push_back(std::make_pair(e.eventIndex, e.nid));
            }
            // save ranks instead of the indices so that the values do not depend on the number of events processed before
            std::sort(pending.begin(), pending.end());
            for (int i = 0; i < (int)pending.size(); i++) {
                myEventIndexCache[pending[i].second] = i;
            }
        };
        rank(myLeaderCars, myInvalidatedLeaderCars);
        for (const Group& g : myGroups) {
            rank(g.leaders, g.invalidated);
        }
        myEventIndexCacheTime = SIMSTEP;
    }
    auto it = myEventIndexCache.find(veh->getNumericalID());
    return it == myEventIndexCache.end() ? -1 : it->second;
}


void
MELoop::removeLeaderCar(MEVehicle* v) {
    if (myThreads > 0) {
        if (!v->isDeferred()) {
            // a deferred vehicle has no pending event that could be invalidated
            myGroups[getGroup(v)].invalidated.push(LeaderEvent(v, 0));
        }
    } else {
        myInvalidatedLeaderCars.push(LeaderEvent(v));
    }
}


int
MELoop::getGroup(const MEVehicle* veh) const {
    const MESegment* const seg = veh->getSegment();
    if (seg != nullptr && !MESegment::isInvalid(seg)) {
        return seg->getGroup();
    }
    return myEdgeGroups[veh->getEdge()->getNumericalID()];
}


void
MELoop::initGroups() {
    myGroupsInitialized = true;
    const OptionsCont& oc = OptionsCont::getOptions();
    myThreads = oc.getInt("meso-threads");
    if (myThreads <= 0) {
        return;
    }
    myVerbose = oc.getBool("verbose");
    myRounds = MAX2(1, oc.getInt("meso-threads.rounds"));
    int numGroups = oc.getInt("meso-threads.groups");
    if (numGroups <= 0) {
        numGroups = 4 * myThreads;
    }
    const double minCutTime = STEPS2TIME(string2time(oc.getString("meso-threads.min-cut-time")));
    // union-find over junctions: an edge belongs to the group of its end junction;
    // edges that can be crossed within minCutTime keep both of their junctions in one group
    std::unordered_map<const MSJunction*, int> index;
    std::vector<const MSJunction*> junctions;
    std::vector<int> parent;
    auto id = [&](const MSJunction * j) {
        auto it = index.find(j);
        if (it != index.end()) {
            return it->second;
        }
        index[j] = (int)junctions.size();
        junctions.push_back(j);
        parent.push_back((int)parent.size());
        return (int)parent.size() - 1;
    };
    auto find = [&](int i) {
        while (parent[i] != i) {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    };
    auto firstSegment = [this](const MSEdge * e) -> const MESegment* {
        return e->getNumericalID() < (int)myEdges2FirstSegments.size() ? myEdges2FirstSegments[e->getNumericalID()] : nullptr;
    };
    int numSegments = 0;
    for (const MSEdge* e : MSEdge::getAllEdges()) {
        const MESegment* first = firstSegment(e);
        if (first == nullptr || e->getFromJunction() == nullptr || e->getToJunction() == nullptr) {
            continue;
        }
        for (const MESegment* s = first; s != nullptr; s = s->getNextSegment()) {
            numSegments++;
        }
        const int from = find(id(e->getFromJunction()));
        const int to = find(id(e->getToJunction()));
        // vehicles may drive faster than the speed limit (speedFactor)
        const double fastest = 1.3 * MAX2(e->getSpeedLimit(), NUMERICAL_EPS);
        if (from != to && (!e->isNormal() || first->getLength() < fastest * minCutTime)) {
            parent[from] = to;
        }
    }
    // clusters of junctions ordered along a Z-order curve, then cut into groups with a similar number of segments
    struct Cluster {
        double x = 0, y = 0, weight = 0;
        int count = 0, group = 0;
    };
    std::map<int, Cluster> clusters;
    for (const MSEdge* e : MSEdge::getAllEdges()) {
        if (firstSegment(e) != nullptr && e->getToJunction() != nullptr) {
            Cluster& c = clusters[find(id(e->getToJunction()))];
            for (const MESegment* s = firstSegment(e); s != nullptr; s = s->getNextSegment()) {
                c.weight += 1;
            }
        }
    }
    double xmin = std::numeric_limits<double>::max(), ymin = xmin, xmax = -xmin, ymax = -xmin;
    for (int i = 0; i < (int)junctions.size(); i++) {
        const Position& p = junctions[i]->getPosition();
        Cluster& c = clusters[find(i)];
        c.x += p.x();
        c.y += p.y();
        c.count++;
        xmin = MIN2(xmin, p.x());
        ymin = MIN2(ymin, p.y());
        xmax = MAX2(xmax, p.x());
        ymax = MAX2(ymax, p.y());
    }
    std::vector<std::pair<unsigned long long, int> > order;
    double total = 0;
    double largest = 0;
    for (auto& item : clusters) {
        const Cluster& c = item.second;
        const unsigned int ix = (unsigned int)(65535 * (c.x / c.count - xmin) / MAX2(xmax - xmin, 1.));
        const unsigned int iy = (unsigned int)(65535 * (c.y / c.count - ymin) / MAX2(ymax - ymin, 1.));
        unsigned long long key = 0;
        for (int b = 0; b < 16; b++) {
            key |= ((unsigned long long)((ix >> b) & 1) << (2 * b)) | ((unsigned long long)((iy >> b) & 1) << (2 * b + 1));
        }
        order.push_back(std::make_pair(key, item.first));
        total += c.weight;
        largest = MAX2(largest, c.weight);
    }
    std::sort(order.begin(), order.end());
    double cumulated = 0;
    for (const auto& item : order) {
        Cluster& c = clusters[item.second];
        c.group = MIN2(numGroups - 1, (int)(cumulated * numGroups / MAX2(total, 1.)));
        cumulated += c.weight;
    }
    myGroups.resize(numGroups);
    for (int i = 0; i < numGroups; i++) {
        myGroups[i].rng = new SumoRNG("meso-group-" + toString(i));
        myGroups[i].rng->setSeed(RandHelper::getSeed() + i);
    }
    myEdgeGroups.assign(MSEdge::getAllEdges().size(), 0);
    int cutEdges = 0;
    int edges = 0;
    for (const MSEdge* e : MSEdge::getAllEdges()) {
        const MSJunction* end = e->getToJunction() != nullptr ? e->getToJunction() : e->getFromJunction();
        if (end == nullptr) {
            continue;
        }
        const int group = clusters[find(id(end))].group;
        myEdgeGroups[e->getNumericalID()] = group;
        if (firstSegment(e) != nullptr) {
            for (MESegment* s = myEdges2FirstSegments[e->getNumericalID()]; s != nullptr; s = s->getNextSegment()) {
                s->setGroup(group, myGroups[group].rng);
            }
            if (e->getFromJunction() != nullptr) {
                edges++;
                if (clusters[find(id(e->getFromJunction()))].group != group) {
                    cutEdges++;
                }
            }
        }
    }
    WRITE_MESSAGEF(TL("Parallel meso: % threads, % groups from % junction clusters (largest % of % segments), % of % edges cross groups."),
                   toString(myThreads), toString(numGroups), toString(clusters.size()), toString(largest), toString(numSegments), toString(cutEdges), toString(edges));
    myWorkerPool = new WorkerPool(myThreads);
}


void
MELoop::saveRNGStates(OutputDevice& out) {
    if (!myGroupsInitialized) {
        initGroups();
    }
    for (int i = 0; i < (int)myGroups.size(); i++) {
        out.openTag(SUMO_TAG_RNGMESOGROUP);
        out.writeAttr(SUMO_ATTR_INDEX, i);
        out.writeAttr(SUMO_ATTR_STATE, RandHelper::saveState(myGroups[i].rng));
        out.closeTag();
    }
}


void
MELoop::loadRNGState(int index, const std::string& state) {
    if (!myGroupsInitialized) {
        initGroups();
    }
    if (index < (int)myGroups.size()) {
        RandHelper::loadState(state, myGroups[index].rng);
    } else {
        WRITE_WARNINGF(TL("Ignoring random number state of meso group % (the simulation uses % groups)."), toString(index), toString(myGroups.size()));
    }
}


void
MELoop::vaporizeCar(MEVehicle* v, MSMoveReminder::Notification reason) {
    int qIdx = 0;
    v->getSegment()->send(v, nullptr, qIdx, MSNet::getInstance()->getCurrentTimeStep(), reason);
    removeLeaderCar(v);
}


MESegment*
MELoop::nextSegment(MESegment* s, MEVehicle* v) {
    if (s != nullptr) { // vehicle is not teleporting
        MESegment* next = s->getNextSegment();
        if (next != nullptr) {
            // ok, the street continues
            return next;
        }
    }
    // we have to check the next edge in the vehicle's route
    const MSEdge* nextEdge = v->succEdge(1);
    if (nextEdge == nullptr) {
        // end of route
        return nullptr;
    }
    if (MSGlobals::gUsingInternalLanes && s != nullptr && s->getEdge().isNormal()) {
        const MSEdge* internal = s->getEdge().getInternalFollowingEdge(nextEdge, v->getVClass());
        if (internal) {
            nextEdge = internal;
        }
    }
    return myEdges2FirstSegments[nextEdge->getNumericalID()];
}


int
MELoop::numSegmentsFor(const double length, const double sLength) {
    int no = (int)floor(length / sLength + 0.5);
    if (no == 0) { // assure there is at least one segment
        return 1;
    } else {
        return no;
    }
}


void
MELoop::buildSegmentsFor(const MSEdge& e, const OptionsCont& oc) {
    const MESegment::MesoEdgeType& edgeType = MSNet::getInstance()->getMesoType(e.getEdgeType());
    const double length = e.getLength();
    const int numSegments = numSegmentsFor(length, edgeType.edgeLength);
    const double slength = length / (double)numSegments;
    MESegment* newSegment = nullptr;
    MESegment* nextSegment = nullptr;
    const bool laneQueue = oc.getBool("meso-lane-queue");
    const bool lift = oc.getBool("meso-ltm");
    bool multiQueue = laneQueue || (oc.getBool("meso-multi-queue") && e.getLanes().size() > 1 && e.getNumSuccessors() > 1);
    for (int s = numSegments - 1; s >= 0; s--) {
        std::string id = e.getID() + ":" + toString(s);
        newSegment = lift
                     ? new MELSegment(id, e, nextSegment, slength, e.getLanes()[0]->getSpeedLimit(), s, multiQueue, edgeType)
                     : new MESegment(id, e, nextSegment, slength, e.getLanes()[0]->getSpeedLimit(), s, multiQueue, edgeType);
        multiQueue = laneQueue;
        nextSegment = newSegment;
    }
    while (e.getNumericalID() >= static_cast<int>(myEdges2FirstSegments.size())) {
        myEdges2FirstSegments.push_back(0);
    }
    myEdges2FirstSegments[e.getNumericalID()] = newSegment;
    for (MSLane* lane : e.getLanes()) {
        lane->updateMesoGUISegments();
    }
}


MESegment*
MELoop::getSegmentForEdge(const MSEdge& e, double pos) {
    if (e.getNumericalID() >= (int)myEdges2FirstSegments.size()) {
        return nullptr;
    }
    MESegment* s = myEdges2FirstSegments[e.getNumericalID()];
    if (pos > 0) {
        double cpos = 0;
        while (s->getNextSegment() != nullptr && cpos + s->getLength() < pos) {
            cpos += s->getLength();
            s = s->getNextSegment();
        }
    }
    return s;
}


bool
MELoop::isEnteringRoundabout(const MSEdge& e) {
    for (const MSEdge* succ : e.getSuccessors()) {
        if (succ->isRoundabout()) {
            return true;
        }
    }
    return false;
}


MELoop::LeaderEvent::LeaderEvent(MEVehicle* v, long long int index):
    time(v->getEventTime()),
    eventIndex(index),
    nid(v->getNumericalID()),
    veh(v) { }


MELoop::LeaderEvent::LeaderEvent(MEVehicle* v):
    time(v->getEventTime()),
    eventIndex(myEventCounter++),
    nid(v->getNumericalID()),
    veh(v) { }

/****************************************************************************/
