#include "urGovernor/ObjectTracker.h"

#include <gtest/gtest.h>

#include <algorithm>
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
            p_tracker = new ObjectTracker();

            vector<Object> ini_objs = { // NOTE listed in descending z-order so it matches top()
                {0,6,5},
                {6,6,4},
                {4,4,3},
                {2,2,2},
                {0,0,1}
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

        void TearDown() override
        {
            delete p_tracker;
        }

        ObjectTracker* p_tracker;
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
    ASSERT_EQ(p_tracker->object_count(), 0);

    p_tracker->update(object_lists[frame]);
    ASSERT_EQ(p_tracker->active_objects(), object_lists[frame]);
}

/* EmptyUpdate
 *      Test sending an empty list of objects
 */
TEST_F(ObjectTrackerTest, EmptyUpdate)
{
    vector<Object> empty_list;
    p_tracker->update(object_lists[frame]);
    p_tracker->update(empty_list);
    ASSERT_EQ(p_tracker->active_objects(), object_lists[frame]);
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
        p_tracker->update(curr_list);
        curr_list = object_lists[frame++];
    }

    ASSERT_EQ(p_tracker->active_objects(), object_lists[frame - 2]);
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
        p_tracker->update(curr_list);
        curr_list = object_lists[frame++];
    }
    
    // Next frame has a point go off screen, should still track for 1 frame
    p_tracker->update(object_lists[frame]);
    ASSERT_EQ(p_tracker->object_count(), object_lists[frame - 2].size());
    frame++;

    // Object should be removed by next frame
    p_tracker->update(object_lists[frame]);
    ASSERT_EQ(p_tracker->object_count(), object_lists[frame - 2].size());
}

/* GetTop
 *      @brief checks that the objects are ordered
 */
TEST_F(ObjectTrackerTest, GetTop)
{
    vector<Object> unsorted_vec =
    {
        {0, 0, 1},
        {2, 2, 5},
        {4, 4, 3}
    };

    vector<Object> sorted_vec = unsorted_vec;
    sort(sorted_vec.begin(), sorted_vec.end(), greater<Object>());

    p_tracker->update(unsorted_vec);
    Object top_obj;
    
    ASSERT_TRUE(p_tracker->top(top_obj));
    ASSERT_EQ(top_obj, sorted_vec[0]);

    ASSERT_EQ(p_tracker->active_objects(), sorted_vec);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
