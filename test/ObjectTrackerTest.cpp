#include "urGovernor/ObjectTracker.h"

#include <gtest/gtest.h>

#include <utility>
#include <vector>

using namespace std;

/* ObjectTrackerTest
 *      Fixture for objecttracker
 */
class ObjectTrackerTest : public ::testing::Test {
    protected:
        void SetUp() override
        {
            /* Initial points
             *     --> x
             *    _____________________
             * |  |o(0,0)        
             * V  |    o(2,2)
             * y  |        o(4,4)
             *    |o(0,6)      o(6,6)
             *    |
             *    |___________________ _
             *
             *    each frame the points move in +'ve y by y_Speed
             */
            vector<Object> ini_objs = {
                {0,0,1},
                {2,2,1},
                {4,4,1},
                {6,6,1},
                {0,6,1}
            };

            for (int i = 0; i <  num_frames; i++)
            {
                object_lists.push_back(ini_objs);
                for (int j = 0; j < ini_objs.size(); j++)
                {
                    ini_objs[j].y += y_speed;
                    if(ini_objs[j].y > y_max)
                        ini_objs.erase(ini_objs.begin() + j--);
                }
            }
        }

        ObjectTracker tracker;
        vector<vector<Object> > object_lists;
        int32_t frame = 0;
        const int32_t y_speed = 1;
        const int32_t y_max = 10;
        const int32_t num_frames = 10;
};


/* EmptyStart
 *      Test sending new objects with an empty active list
 */
TEST_F(ObjectTrackerTest, EmptyStart)
{
    ASSERT_EQ(tracker.object_count(), 0);

    tracker.update(object_lists[frame]);
    ASSERT_EQ(tracker.active_objects(), object_lists[frame]);
}

/* EmptyUpdate
 *      Test sending an empty list of objects
 */
TEST_F(ObjectTrackerTest, EmptyUpdate)
{
    vector<Object> empty_list;
    tracker.update(object_lists[frame]);
    tracker.update(empty_list);
    ASSERT_EQ(tracker.active_objects(), object_lists[frame]);
}

/* GenericUpdate
 *      Test sending multiple frames to confirm its tracked
 */
TEST_F(ObjectTrackerTest, GenericUpdate)
{
    vector<Object> curr_list = object_lists[frame];
    size_t curr_size = curr_list.size();

    // Send all updates up until a point goes off screen
    while(curr_size == curr_list.size())
    {
        tracker.update(curr_list);
        curr_list = object_lists[frame++];
    }

    ASSERT_EQ(tracker.active_objects(), object_lists[frame - 2]);
}

/* DissapearedObjects
 *      Make sure objects out of frame are removed from registry
 *      (Note default number of frames cutoff is 1)
 */
TEST_F(ObjectTrackerTest, DissapearedObjects)
{
    vector<Object> curr_list = object_lists[frame];
    size_t curr_size = curr_list.size();

    // Send all updates up until a point goes off screen
    while(curr_size == curr_list.size())
    {
        tracker.update(curr_list);
        curr_list = object_lists[frame++];
    }
    
    // Next frame has a point go off screen, should still track for 1 frame
    tracker.update(object_lists[frame]);
    ASSERT_EQ(tracker.object_count(), object_lists[frame - 2].size());
    frame++;

    // Object should be removed by next frame
    tracker.update(object_lists[frame]);
    ASSERT_EQ(tracker.object_count(), object_lists[frame - 2].size());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
