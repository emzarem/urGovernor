#include "urGovernor/ObjectTracker.h"

#include <gtest/gtest.h>

#include <utility>
#include <vector>

using namespace std;

TEST(ObjectTrackerTest, EmptyStart)
{
    ObjectTracker tracker;
    Object obj = {1, 1}, obj2 = {2,2};
    vector<Object> object_list = {obj, obj2};

    tracker.update(object_list);
    for (auto itr = tracker.m_active_objects.begin(); itr != tracker.m_active_objects.end(); itr++)
    {
        ASSERT_TRUE(itr->second.x == object_list[itr->first].x &&
                itr->second.y == object_list[itr->first].y);
    }
}

TEST(ObjectTrackerTest, EmptyUpdate)
{
}

TEST(ObjectTrackerTest, CloseUpdates)
{

}

TEST(ObjectTrackerTest, DissapearedObjects)
{

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
