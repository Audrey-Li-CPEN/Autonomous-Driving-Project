#include "scan_matching_skeleton/correspond.h"
#include "cmath"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                            [[maybe_unused]] vector<vector<int>> &jump_table, vector<Correspondence> &c, [[maybe_unused]] float prob)
{
  c.clear();
  const size_t n = trans_points.size();
  const size_t m = old_points.size();
  size_t min_index = 0;
  size_t second_min_index = 0;

  for (size_t ind_trans = 0; ind_trans < n; ++ind_trans)
  {
    float min_dist = 100000.00;
    for (size_t ind_old = 0; ind_old < m; ++ind_old)
    {
      float dist = old_points[ind_trans].distToPoint2(&trans_points[ind_old]);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_index = ind_old;
        if (ind_old == 0)
        {
          second_min_index = ind_old + 1;
        }
        else
        {
          second_min_index = ind_old - 1;
        }
      }
    }
    c.push_back(Correspondence(&trans_points[ind_trans], &points[ind_trans], &old_points[min_index], &old_points[second_min_index]));
  }
}

void getCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                       vector<vector<int>> &jump_table, vector<Correspondence> &c, float prob)
{
    c.clear();
    const int trans_size = trans_points.size();
    const int old_size = old_points.size();

    for (int i = 0; i < trans_size; ++i)
    {
        Point& p = trans_points[i];
        
        // Binary search for the initial closest point
        int left = 0, right = old_size - 1;
        while (left <= right)
        {
            int mid = left + (right - left) / 2;
            if (old_points[mid].theta > p.theta)
            {
                right = mid - 1;
            }
            else
            {
                left = mid + 1;
            }
        }

        int best = right;
        int second_best = left;
        float best_dist = p.distToPoint2(&old_points[best]);
        float second_best_dist = p.distToPoint2(&old_points[second_best]);

        // Fine-tune the search using jump table
        for (int dir = -1; dir <= 1; dir += 2)
        {
            int j = (dir == -1) ? right : left;
            while (j >= 0 && j < old_size)
            {
                float dist = p.distToPoint2(&old_points[j]);
                if (dist < best_dist)
                {
                    second_best = best;
                    second_best_dist = best_dist;
                    best = j;
                    best_dist = dist;
                }
                else if (dist < second_best_dist)
                {
                    second_best = j;
                    second_best_dist = dist;
                }
                j = jump_table[j][dir == -1 ? DOWN_SMALL : UP_SMALL];
            }
        }

        if (best != second_best)
        {
            if (rand() / (float)RAND_MAX > prob) {
                continue;  
            }
            c.push_back(Correspondence(&p, &points[i], &old_points[best], &old_points[second_best]));
        }
    }
}

void computeJump(vector<vector<int>> &table, vector<Point> &points)
{
  table.clear();
  int n = points.size();
  for (int i = 0; i < n; ++i)
  {
    vector<int> v = {n, n, -1, -1};
    for (int j = i + 1; j < n; ++j)
    {
      if (points[j].r < points[i].r)
      {
        v[UP_SMALL] = j;
        break;
      }
    }
    for (int j = i + 1; j < n; ++j)
    {
      if (points[j].r > points[i].r)
      {
        v[UP_BIG] = j;
        break;
      }
    }
    for (int j = i - 1; j >= 0; --j)
    {
      if (points[j].r < points[i].r)
      {
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for (int j = i - 1; j >= 0; --j)
    {
      if (points[j].r > points[i].r)
      {
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
