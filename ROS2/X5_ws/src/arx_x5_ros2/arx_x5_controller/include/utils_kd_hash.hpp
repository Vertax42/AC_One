/**
 * Copyright (c) [2025] XinChengYang <vertax@foxmail.com> <yaphetys@gmail.com>
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once
#include <iostream>
#include <unordered_map>

template <typename data_type = float, typename T = void *>
struct Hash_map_3d {
    using hash_3d_T = std::unordered_map<data_type, std::unordered_map<data_type, std::unordered_map<data_type, T> > >;
    hash_3d_T m_map_3d_hash_map;
    void insert(const data_type &x, const data_type &y, const data_type &z, const T &target)
    {
        m_map_3d_hash_map[x][y][z] = target;
    }

    int if_exist(const data_type &x, const data_type &y, const data_type &z)
    {
        if(m_map_3d_hash_map.find(x) == m_map_3d_hash_map.end())
        {
            return 0;
        } else if(m_map_3d_hash_map[x].find(y) == m_map_3d_hash_map[x].end())
        {
            return 0;
        } else if(m_map_3d_hash_map[x][y].find(z) == m_map_3d_hash_map[x][y].end())
        {
            return 0;
        }
        return 1;
    }

    void clear() { m_map_3d_hash_map.clear(); }

    int total_size()
    {
        int count = 0;
        for(auto it : m_map_3d_hash_map)
        {
            for(auto it_it : it.second)
            {
                for(auto it_it_it : it_it.second)
                {
                    count++;
                }
            }
        }
        return count;
    }
};

template <typename data_type = float, typename T = void *>
struct Hash_map_2d {
    using hash_2d_T = std::unordered_map<data_type, std::unordered_map<data_type, T> >;
    // using hash_2d_it = typename std::unordered_map<data_type,
    // std::unordered_map<data_type, T> >::iterator ; using hash_2d_it_it =
    // typename std::unordered_map<data_type, T>::iterator ;

    hash_2d_T m_map_2d_hash_map;
    void insert(const data_type &x, const data_type &y, const T &target) { m_map_2d_hash_map[x][y] = target; }

    int if_exist(const data_type &x, const data_type &y)
    {
        if(m_map_2d_hash_map.find(x) == m_map_2d_hash_map.end())
        {
            return 0;
        } else if(m_map_2d_hash_map[x].find(y) == m_map_2d_hash_map[x].end())
        {
            return 0;
        }

        return 1;
    }

    void clear() { m_map_2d_hash_map.clear(); }

    int total_size()
    {
        int count = 0;
        // for(hash_2d_it it =  m_map_2d_hash_map.begin(); it !=
        // m_map_2d_hash_map.end(); it++)
        for(auto it : m_map_2d_hash_map)
        {
            for(auto it_it : it.second)
            {
                count++;
            }
        }
        return count;
    }
};
