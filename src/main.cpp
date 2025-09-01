#include <Eigen/Core>
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Window.hpp>
#include <SFML/Window/Event.hpp>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include "algo/raytrace.hpp"
#include "algo/rrt.hpp"
#include "ds/kdtree.hpp"
#include "ds/occupancy_grid.hpp"

using State = Eigen::Vector2f;
typedef bool (*StatePredicate)(const State&);

template<PointSet<2> T>
void run_vis(const T& point_set, const std::vector<size_t>& path, size_t start_id, size_t end_id,
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

        // Draw points
        for (size_t i = 0; i < point_set.size(); i++) {
            const State& p = point_set[i];
            sf::CircleShape circle(5.0);

            float screen_x = (p.x() - grid.origin().x()) * scale_x;
            float screen_y = window.getSize().y - (p.y() - grid.origin().y()) * scale_y;

            circle.setOrigin(circle.getGeometricCenter());
            circle.setPosition(sf::Vector2f(screen_x, screen_y));

            if (i == start_id) {
                circle.setFillColor(sf::Color::Red);
            } else if (i == end_id) {
                circle.setFillColor(sf::Color::Green);
            } else {
                circle.setFillColor(sf::Color::Black);
            }
            window.draw(circle);
        }

        // Draw path
        if (path.size() > 1) {
            for (size_t i = 0; i < path.size() - 1; ++i) {
                const State& p1 = point_set[path[i]];
                const State& p2 = point_set[path[i + 1]];

                sf::Vertex line[] = {
                    sf::Vertex({(p1.x() - grid.origin().x()) * scale_x,
                                   window.getSize().y - (p1.y() - grid.origin().y()) * scale_y},
                        sf::Color::Black),
                    sf::Vertex({(p2.x() - grid.origin().x()) * scale_x,
                                   window.getSize().y - (p2.y() - grid.origin().y()) * scale_y},
                        sf::Color::Black)};
                window.draw(line, 2, sf::PrimitiveType::Lines);
            }
        }

        window.display();
    }
}

static const State START(0.0, 0.0);
static const State GOAL(0.2, 0.0);
static const size_t NUM_POINTS = 10000;
static constexpr float MOVE_DIST = 0.01;
static const State MIN_BOUND(-0.1, -0.1);
static const State MAX_BOUND(0.3, 0.1);
static constexpr float SQ_GOAL_TOL = 0.0001;
static constexpr float GRID_RESOLUTION = 0.05;

int main() {
    KDTree<2, 4> kd_tree;

    std::random_device rd;
    std::mt19937 gen(rd());

    State diff = MAX_BOUND - MIN_BOUND;
    size_t x_cells = static_cast<size_t>(std::ceil(diff.x() / GRID_RESOLUTION));
    size_t y_cells = static_cast<size_t>(std::ceil(diff.y() / GRID_RESOLUTION));
    bool* storage = new bool[x_cells * y_cells]{};
    OccupancyGridView grid_view(storage, x_cells, y_cells, MIN_BOUND, GRID_RESOLUTION);

    auto path_ids = rrt<2>(
        START, GOAL, NUM_POINTS, MOVE_DIST, MIN_BOUND, MAX_BOUND, SQ_GOAL_TOL, kd_tree,
        [&](const State& a, const State& b) { return is_segment_occupied(a, b, grid_view); }, gen);

    run_vis(kd_tree, path_ids, 0, kd_tree.size() - 1, grid_view);

    return 0;
}
