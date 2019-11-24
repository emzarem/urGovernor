/* @file ObjectTracker.cpp
 *      @author emzarem
 *      
 *      This class handles the following functionality:
 *              -> Maintains sorted list of 'active' weeds
 *              -> Removes weeds from the list once they are out of scope
 *              -> Chooses the next weed to target
 */

#include "urGovernor/ObjectTracker.h"
#include <algorithm>    // sort
#include <numeric>      // iota
#include <math.h>       // sqrt

/* Distance
 *      @brief type used for centroid distances
 */
typedef float Distance;

/* euclidean_distance
 *      @brief Calculates euclidean distance between two objects
 */
static Distance euclidean_distance(const Object& a, const Object& b)
{
    Distance delt_x = a.x - b.x;
    Distance delt_y = a.y - b.y;
    Distance delt_z = a.z - b.z;
    return sqrt(delt_x*delt_x + delt_y*delt_y + delt_z*delt_z);
}

/* ObjectTracker
 *      @brief constructor
 *
 *      @param max_dissapeared_frms : 
 *              max number of missed frames before object removed
 */
ObjectTracker::ObjectTracker(uint32_t max_dissapeared_frms) :
    m_max_dissapeared_frms(max_dissapeared_frms)
{}

/* ~ObjectTracker
 *      @brief destructor
 */
ObjectTracker::~ObjectTracker()
{}

/* active_objects
 *      @brief returns vector of active objects
 */
std::vector<Object> ObjectTracker::active_objects()
{
    std::vector<Object> to_ret;

    for (auto itr = m_active_objects.begin(); itr != m_active_objects.end(); itr++)
    {
        to_ret.push_back(itr->second);
    }

    return to_ret;
}

/* object_count
 *      @brief returns number of currently tracked objects
 */
size_t ObjectTracker::object_count()
{
    return m_active_objects.size();
}

/* update
 *      @brief updates the active list of objects
 *
 *      @details
 *              1) calculates a distance matrix between current objects and new:
 *                             __  new_objects --->  __
 *             active_objects | D_1,1 . . . . . . D_1,n|
 *                  |         |  .       .             |
 *                  |         |  .           .         |
 *                  |         | D_m,1             D_m,n|
 *                  V         |__                    __|
 *
 *               2) sorts through each row to have min in 1st col
 *                     -Then the rows are sorted by the min distance in each
 *
 *      @param new_objs : list of centroids
 */
void ObjectTracker::update(const std::vector<Object>& new_objs)
{
    if (new_objs.size() == 0) 
    {
        // If we didn't get any new objects, all are dissapeared
        for (auto itr = m_dissapeared.begin(); itr != m_dissapeared.end(); itr++)
        {
            m_dissapeared[itr->first]++;
        }
    }

    else if (m_active_objects.size() == 0)
    {
        // If we don't have any current objects register all the new ones
        for (auto itr = new_objs.begin(); itr != new_objs.end(); itr++)
        {
            register_object(*itr);
        }
    }

    else
    {
        /* 1. Calculate distance between each current point and each new point */
        int32_t m = m_active_objects.size();
        int32_t n = new_objs.size();
        std::vector< std::vector<Distance> > dist_matrix(m, std::vector<Distance>(n));

        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++) 
            {
                Object crnt_obj = m_active_objects[m_id_list[i]];
                dist_matrix[i][j] = euclidean_distance(crnt_obj, new_objs[j]);
            }
        }

        /* 2. Sort by distance */  
        // Keep track of indices -- new objs
        // | 0 1 - - - - - n |
        // | 0 1 - - - - - n |
        // | - - - - - - - - |
        // | - - - - - - - - |
        // | 0 1 - - - - - n |
        std::vector<std::vector<size_t> > sorted_ids(m, std::vector<size_t>(n));
        for (int i = 0; i < m; i++)
            std::iota(sorted_ids[i].begin(), sorted_ids[i].end(), 0);

        // Keep track of indices -- active objs
        // | 0 1 - - - - - m |
        std::vector<size_t> active_obj_ids(m);
        std::iota(active_obj_ids.begin(), active_obj_ids.end(), 0);

        // Sort intra-row (find closest new object to each old object)
        // | 0 1 - <---> - n |
        // | 0 1 - <---> - n |
        // | - - - <---> - - |
        // | - - - <---> - - |
        // | 0 1 - <---> - n |
        for (int i = 0; i < m; i++)
        {
             std::sort(sorted_ids[i].begin(), sorted_ids[i].end(), 
                    [&dist_matrix, &i](const size_t& a, const size_t& b) -> bool
                    {
                        return dist_matrix[i][a] < dist_matrix[i][b];
                    });
        }
        
        // Sort the rows based on min distance in each row
        // | 0 1 - <---> - m |
        std::sort(active_obj_ids.begin(), active_obj_ids.end(), 
                [&dist_matrix, &sorted_ids](const size_t& a, const size_t& b) -> bool
                {
                    return dist_matrix[a][sorted_ids[a][0]] < dist_matrix[b][sorted_ids[b][0]];
                });
        
        /* 3. Find matching objects */
        std::map<size_t, int> used_cols;

        // Loop over each active object
        for (auto itr = active_obj_ids.begin(); itr != active_obj_ids.end(); itr++)
        {
            bool found_update = false;

            // loop over each new object in the row 
            for (auto sub_itr = sorted_ids[*itr].begin(); sub_itr != sorted_ids[*itr].end(); sub_itr++)
            {
                if (used_cols.count(*sub_itr) == 0)
                {
                    // We found our tracked object!
                    used_cols[*sub_itr] = 1; // update that we used this object
                    m_active_objects[m_id_list[*itr]] = new_objs[*sub_itr];
                    found_update = true;
                    break;
                }

            }

            // If not updated its missing this frame
            if (!found_update)
                m_dissapeared[m_id_list[*itr]]++;
        }

    }

    /* 4. Update missing objects */
    cleanup_dissapeared();
}

/* register_object
 *      @brief registers an object as active
 */
ObjectID ObjectTracker::register_object(const Object& obj)
{
    m_active_objects[m_next_id] = obj;
    m_dissapeared[m_next_id] = 0;
    m_id_list.push_back(m_next_id);
    return m_next_id++;
}

/* deregister_object
 *      @brief removes object with id from registry
 */
void ObjectTracker::deregister_object(const ObjectID id)
{
    m_active_objects.erase(id);
    m_dissapeared.erase(id);
    for (auto itr = m_id_list.begin(); itr != m_id_list.end(); itr++)
    {
        if (*itr == id)
        {
            m_id_list.erase(itr);
            break;
        }
    }
}


/* cleanup_dissapeared
 *      @brief removes any objects whose dissapeared value > max_dissapeared
 */
void ObjectTracker::cleanup_dissapeared()
{
    for (auto itr = m_dissapeared.begin(); itr != m_dissapeared.end(); itr++)
    {
        if (itr->second > m_max_dissapeared_frms)
            deregister_object(itr->first);
    }
}










