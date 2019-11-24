/* @file ObjectTracker.h
 *      @author emzarem
 *      
 *      This class handles the following functionality:
 *              -> Maintains sorted list of 'active' weeds
 *              -> Removes weeds from the list once they are out of scope
 *              -> Chooses the next weed to target
 */

#ifndef TRACKER_H
#define TRACKER_H

#include <gtest/gtest.h>

#include <map>
#include <vector>

/* Object
 *      @brief Struct holding centroid of object to track
 */
struct Object {
    int32_t x;
    int32_t y;
    int32_t z;
};

bool operator==(const Object& lhs, const Object& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator>(const Object& lhs, const Object& rhs)
{
    // Currently using z value as basis for sorting (use for size)
    return lhs.z > rhs.z;
}


/* ObjectID
 *      @brief Object ID type used to track
 */
typedef uint32_t ObjectID;

/* ObjectTracker
 *      @brief Class to perform centroid tracking given bounding boxes
 *
 *      @note Constructor accepts both number of missing frames before removing object
 *            and 
 */
class ObjectTracker {
    public:
        ObjectTracker(uint32_t max_dissapeared_frms = 1);
        //TODO add option to specify comparator to keep list of objects sorted
        //ObjectTracker(uint32_t max_dissapeared_frms = 1, bool (*f)(const Object&, const Object&));
        ~ObjectTracker();
        
        std::vector<Object> active_objects();
        size_t object_count();
        void update(const std::vector<Object>& new_objs);

    private:
        ObjectID register_object(const Object& obj);
        void deregister_object(const ObjectID id);
        void cleanup_dissapeared();

    private:
        ObjectID m_next_id;
        uint32_t m_max_dissapeared_frms;

        std::map<ObjectID, Object> m_active_objects;
        std::map<ObjectID, uint32_t> m_dissapeared;

        std::vector<ObjectID> m_id_list;
};

#endif
