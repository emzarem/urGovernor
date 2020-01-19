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
#include <sstream>
#include <vector>

/* Object
 *      @brief Struct holding centroid of object to track
 */
struct Object {
    // All values in cm
    float x;
    float y;
    float z;
    float size;

    operator std::string() const
    {
        std::ostringstream oss;
        oss << "Object " << this << ":" << std::endl
            << "    x: " << this->x << std::endl
            << "    y: " << this->y << std::endl
            << "    z: " << this->z << std::endl;
        return oss.str();
    }
};

inline bool operator==(const Object& lhs, const Object& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.size == rhs.size;
}

inline bool operator>(const Object& lhs, const Object& rhs)
{
    // Currently using size value as a basis for sorting
    return lhs.size > rhs.size;
}


/* ObjectID
 *      @brief Object ID type used to track
 */
typedef uint32_t ObjectID;
/* Distance
 *      @brief type used for centroid distances
 */
typedef float Distance;


/* ObjectTracker
 *      @brief Class to perform centroid tracking given bounding boxes
 *
 *      @note Constructor accepts both number of missing frames before removing object
 *            and 
 */
class ObjectTracker {
    public:
        ObjectTracker(Distance distTol, uint32_t max_dissapeared_frms = 1 ,uint32_t min_valid_framecount = 1);
        ~ObjectTracker();
       
        // Getters
        std::vector<Object> active_objects();
        size_t object_count();
        
        // Sorted operations
        bool top(Object& to_ret);

        // Additional checks:
        // Is it uprooted? How many frames has it been in?
        bool topValid(Object& to_ret);

        // Modifiers
        void update(const std::vector<Object>& new_objs);

    private:
        ObjectID register_object(const Object& obj);
        void deregister_object(const ObjectID id);
        void cleanup_dissapeared();

    private:
        Distance m_dist_tol;

        ObjectID m_next_id;
        /* Max num. frames an object can disappear in before being deregistered */
        uint32_t m_max_dissapeared_frms;
        /* Min num. of consecutive frames to be considered valid */
        uint32_t m_min_framecount;

        std::map<ObjectID, Object> m_active_objects;

        /* Each registered object will have an associated:
         *      framecount: the number of consecutive frames this object has appeared in
         *      dissapeared: total number of frames this alleged "object" has not appeared in
         *      uprooted: true if this weed has already been processed  
         */
        std::map<ObjectID, uint32_t> m_framecount;
        std::map<ObjectID, uint32_t> m_disappeared;
        std::map<ObjectID, bool> m_uprooted;

        std::vector<ObjectID> m_id_list;
};

#endif
