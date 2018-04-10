/* 
 * grid_draw.hpp
 * 
 * Created on: Apr 09, 2018 23:13
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GRID_DRAW_HPP
#define GRID_DRAW_HPP

#include <cstdint>

#include "lightviz/cv_draw.hpp"

#include "decomp/square_grid.hpp"
#include "graph/graph.hpp"

namespace librav
{
struct GridDraw
{
    GridDraw(int32_t ppu) : pixel_per_unit(ppu) {}

    // basic drawing
    template <typename GridType>
    cv::Mat CreateSquareGridCanvas(GridType *grid)
    {
        // create canvas
        int32_t vis_side_size = grid->GetCellSize() * pixel_per_unit;
        cv::Mat canvas(grid->SizeY() * vis_side_size, grid->SizeX() * vis_side_size, CV_8UC3, LVColors::bg_color);

        return canvas;
    }

    template <typename GridType>
    cv::Mat DrawSquareGridCell(cv::Mat canvas, GridType *grid)
    {
        // draw cells
        for (int32_t y = 0; y < grid->SizeY(); ++y)
            for (int32_t x = 0; x < grid->SizeX(); ++x)
            {
                auto cell = grid->GetCell(x, y);
                if (cell->label == SquareCellLabel::OCCUPIED)
                {
                    cv::Range rngx(cell->vertices[0].x * pixel_per_unit, cell->vertices[1].x * pixel_per_unit);
                    cv::Range rngy(cell->vertices[0].y * pixel_per_unit, cell->vertices[2].y * pixel_per_unit);
                    canvas(rngy, rngx) = LVColors::obs_color;
                }
            }

        return canvas;
    }

    template <typename GridType>
    cv::Mat DrawSquareGridCost(cv::Mat canvas, GridType *grid)
    {
        // draw cells
        for (int32_t y = 0; y < grid->SizeY(); ++y)
            for (int32_t x = 0; x < grid->SizeX(); ++x)
            {
                auto cell = grid->GetCell(x, y);
                if (cell->label != SquareCellLabel::OCCUPIED)
                {
                    cv::Range rngx(cell->vertices[0].x * pixel_per_unit, cell->vertices[1].x * pixel_per_unit);
                    cv::Range rngy(cell->vertices[0].y * pixel_per_unit, cell->vertices[2].y * pixel_per_unit);
                    canvas(rngy, rngx) = LightViz::JetPaletteTransform(cell->cost_map);
                }
            }

        return canvas;
    }

    template <typename GridType>
    cv::Mat DrawSquareGridNet(cv::Mat canvas, GridType *grid)
    {
        // draw horizontal lines
        for (int32_t y = 1; y < grid->SizeY(); ++y)
        {
            auto first_vertex = grid->GetCell(0, y)->vertices[0];
            auto last_vertex = grid->GetCell(grid->SizeX() - 1, y)->vertices[1];

            cv::Point pt1(first_vertex.x * pixel_per_unit, first_vertex.y * pixel_per_unit);
            cv::Point pt2(last_vertex.x * pixel_per_unit, last_vertex.y * pixel_per_unit);

            LightViz::DrawLine(canvas, pt1, pt2);
        }

        // draw vertical lines
        for (int32_t x = 1; x < grid->SizeX(); ++x)
        {
            auto first_vertex = grid->GetCell(x, 0)->vertices[0];
            auto last_vertex = grid->GetCell(x, grid->SizeY() - 1)->vertices[2];

            cv::Point pt1(first_vertex.x * pixel_per_unit, first_vertex.y * pixel_per_unit);
            cv::Point pt2(last_vertex.x * pixel_per_unit, last_vertex.y * pixel_per_unit);

            LightViz::DrawLine(canvas, pt1, pt2);
        }

        return canvas;
    }

    template <typename GridCellType>
    cv::Mat DrawSquareGridPathStartGoal(cv::Mat canvas, const std::vector<GridCellType *> &path)
    {
        cv::Range srngx(path.front()->vertices[0].x * pixel_per_unit, path.front()->vertices[1].x * pixel_per_unit);
        cv::Range srngy(path.front()->vertices[0].y * pixel_per_unit, path.front()->vertices[2].y * pixel_per_unit);
        canvas(srngy, srngx) = LVColors::start_color;

        cv::Range grngx(path.back()->vertices[0].x * pixel_per_unit, path.back()->vertices[1].x * pixel_per_unit);
        cv::Range grngy(path.back()->vertices[0].y * pixel_per_unit, path.back()->vertices[2].y * pixel_per_unit);
        canvas(grngy, grngx) = LVColors::finish_color;

        return canvas;
    }

    template <typename GridCellType>
    cv::Mat DrawSquareGridPath(cv::Mat canvas, const std::vector<GridCellType *> &path)
    {
        for (int i = 0; i < path.size() - 1; ++i)
        {
            auto first_cell = path[i]->center;
            auto next_cell = path[i + 1]->center;

            cv::Point pt1(first_cell.x * pixel_per_unit, first_cell.y * pixel_per_unit);
            cv::Point pt2(next_cell.x * pixel_per_unit, next_cell.y * pixel_per_unit);

            LightViz::DrawLine(canvas, pt1, pt2, LVColors::intermediate_color);
        }

        return canvas;
    }

    // graph related drawing
    template <typename GridType, typename GridCellType>
    cv::Mat DrawSquareGridGraph(cv::Mat canvas, GridType *grid, Graph_t<GridCellType *> *graph)
    {
        // draw all edges
        auto edges = graph->GetAllEdges();
        for (auto &edge_it : edges)
        {
            auto edge = *edge_it;
            int64_t loc_x1 = edge.src_->state_->center.x * pixel_per_unit;
            int64_t loc_y1 = edge.src_->state_->center.y * pixel_per_unit;
            int64_t loc_x2 = edge.dst_->state_->center.x * pixel_per_unit;
            int64_t loc_y2 = edge.dst_->state_->center.y * pixel_per_unit;

            LightViz::DrawLine(canvas, cv::Point(loc_x1, loc_y1), cv::Point(loc_x2, loc_y2), cv::Scalar(237, 149, 100));
        }

        // draw all vertices
        for (auto vertex = graph->vertex_begin(); vertex != graph->vertex_end(); ++vertex)
        {
            // current vertex center coordinate
            int32_t loc_x = vertex->state_->center.x * pixel_per_unit;
            int32_t loc_y = vertex->state_->center.y * pixel_per_unit;

            cv::Point center(loc_x, loc_y);
            LightViz::DrawPoint(canvas, center);

            // if (show_id && vertex->state_->GetUniqueID() % 5 == 0)
            // {
            //     std::string id = std::to_string(vertex->state_->GetUniqueID());
            //     cv::putText(dst, id, cv::Point(loc_x, loc_y), CV_FONT_NORMAL, 0.5, cv::Scalar(204, 204, 102), 1, 1);
            // }
        }

        return canvas;
    }

    int32_t pixel_per_unit = 100;
};
}

#endif /* GRID_DRAW_HPP */