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
/// @file    MELoop.h
/// @author  Daniel Krajzewicz
/// @date    Tue, May 2005
///
// The main mesocopic simulation loop
/****************************************************************************/
#pragma once
#include <config.h>

#include <vector>
#include <queue>
#include <map>
#include <utils/common/SUMOTime.h>
#include <utils/common/RandHelper.h>
#ifdef HAVE_FOX
#include <utils/foxtools/MFXWorkerThread.h>
#endif
#include <microsim/MSMoveReminder.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MESegment;
class MEVehicle;
class MSEdge;
class MSLink;
class MSVehicleControl;
class OptionsCont;
class OutputDevice;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MELoop
 * @brief The main mesocopic simulation loop
 */
class MELoop {
public:
    /// SUMO constructor
    MELoop(const SUMOTime recheckInterval);

    ~MELoop();

    /** @brief Perform simulation up to the given time
     *
     * Checks all vehicles with an event time less or equal than the given time.
     *
     * @param[in] tMax the end time for the sim step
     */
    void simulate(SUMOTime tMax);

    /** @brief Adds the given car to the leading vehicles
     *
     * @param[in] veh the car which became a leading one
     * @param[in] link the link on which the car shall register its approach
     */
    void addLeaderCar(MEVehicle* veh, MSLink* link);

    /** @brief Removes the given car from the leading vehicles
     *
     * @param[in] v the car which was a leading one
     */
    void removeLeaderCar(MEVehicle* v);

    /** @brief Returns the rank of the vehicle's pending event among all pending events (-1 if it has none)
     *
     * Events with the same time are processed in the order in which they were added.
     * The rank is saved in the state so that this order can be restored.
     */
    long long int getEventIndex(const MEVehicle* veh) const;

    /** @brief remove the given car and clean up the relevant data structures */
    void vaporizeCar(MEVehicle* v, MSMoveReminder::Notification reason);

    /// @brief whether the given edge is entering a roundabout
    static bool isEnteringRoundabout(const MSEdge& e);

    /** @brief Compute number of segments per edge (best value stay close to the configured segment length) */
    static int numSegmentsFor(const double length, const double slength);

    /** @brief Build the segments for a given edge
     *
     * @param[in] e the edge to build for
     */
    void buildSegmentsFor(const MSEdge& e, const OptionsCont& oc);

    /** @brief Get the segment for a given edge at a given position
     *
     * @param[in] e the edge to get the segment for
     * @param[in] pos the position to get the segment for
     * @return The relevant segment
     */
    MESegment* getSegmentForEdge(const MSEdge& e, double pos = 0);

    /** @brief change to the next segment
     * this handles combinations of the following cases:
     * (ending / continuing route) and (leaving segment / finishing teleport)
     */
    SUMOTime changeSegment(MEVehicle* veh, SUMOTime leaveTime, MESegment* const toSegment,
                           MSMoveReminder::Notification reason, const bool ignoreLink = false) const;

    /** @brief Remove all vehicles before quick-loading state */
    void clearState();

    /** @brief Split the network into groups whose events are processed in parallel (option --meso-threads)
     *
     * Every edge belongs to the group of the junction it ends at, so all vehicles which want
     *  to pass a junction or enter an edge are handled by the same group.
     */
    void buildGroups(const OptionsCont& oc);

    /// @brief save / load the states of the random number generators of the groups
    void saveRNGStates(OutputDevice& out);
    void loadRNGState(int index, const std::string& state);

private:
    /** @brief Check whether the vehicle may move
     *
     * This method is called when the vehicle reaches its event time and checks
     *  whether it may proceed to the next segment.
     * When called for a group (see buildGroups), moves into the segment of another group
     *  and teleports are deferred to the end of the simulation step.
     *
     * @param[in] veh The vehicle to check
     * @param[in] group The group processing the event or -1
     */
    void checkCar(MEVehicle* veh, const int group = -1);

    /** @brief Retrieve next segment
     *
     * If the segment is not the last on the current edge, its successor is returned.
     *  Otherwise, the first segment of the edge at which the vehicle continues
     *  his journey is returned.
     *
     * @param[in] s The segment the vehicle is currently at
     * @param[in] v The vehicle to get the next segment for
     * @return The vehicle's next segment
     * @todo Recheck the "quick and dirty" stuff (@see MESegment::saveState, @see MESegment::loadState)
     */
    MESegment* nextSegment(MESegment* s, MEVehicle* v);


    /** @brief teleports a vehicle or continues a teleport
     * @param[in] veh The vehicle to teleport
     * @param[in] toSegment The first segment where the vehicle may reenter the network
     */
    void teleportVehicle(MEVehicle* veh, MESegment* const toSegment, bool disconnected);

private:

    struct LeaderEvent {
        LeaderEvent(MEVehicle* v);
        LeaderEvent(MEVehicle* v, long long int index);

        SUMOTime time;
        // eventIndex ensures stable sorting of vehicles with the same event time
        long long int eventIndex;
        // nid allows safely comparing events for deleted vehicles (myInvalidatedLeaderCars)
        long long int nid;
        MEVehicle* veh;

        bool operator<(const LeaderEvent& b) const {
            if (time == b.time) {
                return eventIndex > b.eventIndex;
            }
            return time > b.time;
        }

        bool operator==(const LeaderEvent& b) const {
            return time == b.time && nid == b.nid;
        }

        static long long int myEventCounter;
    };

    /// @brief leader cars in the segments sorted by exit time
    typedef std::priority_queue<LeaderEvent> LeaderEventQeue;
    LeaderEventQeue myLeaderCars;
    LeaderEventQeue myInvalidatedLeaderCars;

    /// @brief check whether the current car was invalidated and remove it from the invalidated cars
    static bool wasInvalidated(const LeaderEvent& e, LeaderEventQeue& invalidated);

    /// @brief check all cars with an event time up to tMax (group -1 for the queues without groups)
    void processEvents(LeaderEventQeue& leaders, LeaderEventQeue& invalidated, SUMOTime tMax, const int group);

    /// @brief the group whose queue holds the event of the given car
    int getGroup(const MEVehicle* veh) const;

    /// @brief a set of junctions with the edges ending there, see buildGroups
    struct Group {
        Group(const std::string& id) : rng(id) {}
        LeaderEventQeue leaders;
        LeaderEventQeue invalidated;
        long long int eventCounter = 0;
        SumoRNG rng;
        /// @brief cars which enter the segment of another group or teleport at the end of the step
        std::vector<MEVehicle*> deferred;
    };
    std::vector<Group> myGroups;

    /// @brief the group of each edge by numerical id
    std::vector<int> myEdgeGroups;

    /// @brief the end of the current simulation step
    SUMOTime myStepEnd = 0;

#ifdef HAVE_FOX
    /// @brief the task processing the events of one group
    class GroupTask : public MFXWorkerThread::Task {
    public:
        GroupTask(MELoop& loop, int group) : myLoop(loop), myGroup(group) {}
        void run(MFXWorkerThread* /*context*/) {
            Group& g = myLoop.myGroups[myGroup];
            myLoop.processEvents(g.leaders, g.invalidated, myLoop.myStepEnd, myGroup);
        }
    private:
        MELoop& myLoop;
        const int myGroup;
    private:
        /// @brief Invalidated assignment operator.
        GroupTask& operator=(const GroupTask&) = delete;
    };
    std::vector<GroupTask*> myGroupTasks;
    MFXWorkerThread::Pool myThreadPool;
#endif

    /// @brief ranks of the pending events by numerical vehicle id (computed once per saving step)
    mutable std::map<long long int, long long int> myEventIndexCache;
    mutable SUMOTime myEventIndexCacheTime = -1;

    /// @brief mapping from internal edge ids to their initial segments
    std::vector<MESegment*> myEdges2FirstSegments;

    /// @brief the interval at which to recheck at full segments (<=0 means asap)
    const SUMOTime myFullRecheckInterval;

    /// @brief the interval at which to recheck at blocked junctions (<=0 means asap)
    const SUMOTime myLinkRecheckInterval;

private:
    /// @brief Invalidated copy constructor.
    MELoop(const MELoop&);

    /// @brief Invalidated assignment operator.
    MELoop& operator=(const MELoop&);
};
