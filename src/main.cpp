#include <Eigen/Core>
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Window.hpp>
#include <SFML/Window/Event.hpp>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <random>
#include <vector>

#include "algo/raytrace.hpp"
#include "algo/rrt.hpp"
#include "ds/kdtree.hpp"
#include "ds/occupancy_grid.hpp"

using State = Eigen::Vector2f;
typedef bool (*StatePredicate)(const State&);

static const State START(0.0, 0.0);
static const State GOAL(0.05, 0.0);
static const size_t NUM_POINTS = 1000;
static constexpr float MOVE_DIST = 0.01;
static const State MIN_BOUND(-0.1, -0.1);
static const State MAX_BOUND(0.1, 0.1);
static constexpr float SQ_GOAL_TOL = 0.0001;
static constexpr float GRID_RESOLUTION = 0.05;
static constexpr float CIRCLE_RADIUS = 5.0;

template<PointSet<2> T>
void run_vis(const T& point_set, const RRTResult& result, const Eigen::Vector2f& real_goal,
    const OccupancyGridView& grid) {
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(800, 600)), "RRT Visualization");
    window.setFramerateLimit(60);

    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            } else if (const auto* resized = event->getIf<sf::Event::Resized>()) {
                sf::FloatRect visibleArea({0.f, 0.f}, sf::Vector2f(resized->size));
                window.setView(sf::View(visibleArea));
            }
        }

        // Calculate scaling factors
        float scale_x = window.getSize().x / grid.real_size().x();
        float scale_y = window.getSize().y / grid.real_size().y();

        window.clear(sf::Color::White);

        // Draw occupancy grid
        const auto [x_cells, y_cells] = grid.size();
        for (size_t y = 0; y < y_cells; y++) {
            for (size_t x = 0; x < x_cells; x++) {
                if (grid.cell(x, y)) {
                    sf::RectangleShape rect;
                    rect.setSize({grid.resolution() * scale_x, grid.resolution() * scale_y});
                    rect.setPosition({(x * grid.resolution()) * scale_x,
                        window.getSize().y - ((y + 1) * grid.resolution()) * scale_y});
                    rect.setFillColor(sf::Color(128, 128, 128));  // Gray
                    window.draw(rect);
                }
            }
        }

        // Draw tree
        for (size_t i = 0; i < point_set.size(); i++) {
            const State& p1 = point_set[i];
            for (size_t j: result.tree[i]) {
                const State& p2 = point_set[j];

                sf::Vertex line[] = {
                    sf::Vertex({(p1.x() - grid.origin().x()) * scale_x,
                                   window.getSize().y - (p1.y() - grid.origin().y()) * scale_y},
                        sf::Color(200, 200, 200)),
                    sf::Vertex({(p2.x() - grid.origin().x()) * scale_x,
                                   window.getSize().y - (p2.y() - grid.origin().y()) * scale_y},
                        sf::Color(220, 220, 220))};
                window.draw(line, 2, sf::PrimitiveType::Lines);
            }
        }

        // Draw path
        if (result.path) {
            const auto& path = *result.path;
            for (size_t i = 0; i < path.size() - 1; i++) {
                const State& p1 = point_set[path[i]];
                const State& p2 = point_set[path[i + 1]];

                sf::Vertex line[] = {
                    sf::Vertex({(p1.x() - grid.origin().x()) * scale_x,
                                   window.getSize().y - (p1.y() - grid.origin().y()) * scale_y},
                        sf::Color::Blue),
                    sf::Vertex({(p2.x() - grid.origin().x()) * scale_x,
                                   window.getSize().y - (p2.y() - grid.origin().y()) * scale_y},
                        sf::Color::Blue)};
                window.draw(line, 2, sf::PrimitiveType::Lines);
            }
        }

        // Draw points
        for (size_t i = 0; i < point_set.size(); i++) {
            const State& p = point_set[i];
            sf::CircleShape circle(CIRCLE_RADIUS);

            float screen_x = (p.x() - grid.origin().x()) * scale_x;
            float screen_y = window.getSize().y - (p.y() - grid.origin().y()) * scale_y;

            circle.setOrigin(circle.getGeometricCenter());
            circle.setPosition(sf::Vector2f(screen_x, screen_y));

            if (i == result.start_idx) {
                circle.setFillColor(sf::Color::Red);
            } else {
                circle.setFillColor(sf::Color::Black);
            }
            window.draw(circle);
        }

        // Draw true goal
        sf::CircleShape end_circle(CIRCLE_RADIUS);
        float end_screen_x = (real_goal.x() - grid.origin().x()) * scale_x;
        float end_screen_y = window.getSize().y - (real_goal.y() - grid.origin().y()) * scale_y;
        end_circle.setOrigin(end_circle.getGeometricCenter());
        end_circle.setPosition(sf::Vector2f(end_screen_x, end_screen_y));
        end_circle.setFillColor(sf::Color::Green);
        window.draw(end_circle);

        window.display();
    }
}

int main() {
    KDTree<2, 4> kd_tree;

    std::random_device rd;
    auto seed = rd();
    std::mt19937 gen(seed);

    std::cout << "Seed: " << seed << "\n";

    State diff = MAX_BOUND - MIN_BOUND;
    size_t x_cells = static_cast<size_t>(std::ceil(diff.x() / GRID_RESOLUTION));
    size_t y_cells = static_cast<size_t>(std::ceil(diff.y() / GRID_RESOLUTION));

    bool* storage = new bool[x_cells * y_cells]{};
    OccupancyGridView grid_view(storage, x_cells, y_cells, MIN_BOUND, GRID_RESOLUTION);
    grid_view.cell(0, 0) = true;

    auto result = rrt<2>(
        START, GOAL, NUM_POINTS, MOVE_DIST, MIN_BOUND, MAX_BOUND, SQ_GOAL_TOL, kd_tree,
        [&](const State& a, const State& b) { return !is_segment_occupied(a, b, grid_view); }, gen);

    run_vis(kd_tree, result, GOAL, grid_view);

    delete[] storage;

    return 0;
}
