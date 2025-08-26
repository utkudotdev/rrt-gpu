#include <Eigen/Core>
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Window.hpp>
#include <SFML/Window/Event.hpp>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <random>
#include <vector>

#include "algo/rrt.hpp"
#include "ds/kdtree.hpp"

using State = Eigen::Vector2f;
typedef bool (*StatePredicate)(const State&);

template<PointSet<2> T>
void run_vis(const T& point_set, const std::vector<size_t>& path, size_t start_id, size_t end_id,
    const State& min_bound, const State& max_bound) {
    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(800, 600)), "RRT Visualization");
    window.setFramerateLimit(60);

    // Print screen corner coordinates
    std::cout << "Screen corners correspond to world points:\n";
    std::cout << "Top-left (0,0): (" << min_bound.x() << ", " << max_bound.y() << ")\n";
    std::cout << "Bottom-right (800,600): (" << max_bound.x() << ", " << min_bound.y() << ")\n";
    std::cout << "Path " << (path.empty() ? "not " : "") << "found\n";

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
        float scale_x = window.getSize().x / (max_bound.x() - min_bound.x());
        float scale_y = window.getSize().y / (max_bound.y() - min_bound.y());

        window.clear(sf::Color::White);

        // Draw points
        for (size_t i = 0; i < point_set.size(); i++) {
            const State& p = point_set[i];
            sf::CircleShape circle(5.0);

            float screen_x = (p.x() - min_bound.x()) * scale_x;
            float screen_y = window.getSize().y - (p.y() - min_bound.y()) * scale_y;

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
                    sf::Vertex({(p1.x() - min_bound.x()) * scale_x,
                                   window.getSize().y - (p1.y() - min_bound.y()) * scale_y},
                        sf::Color::Black),
                    sf::Vertex({(p2.x() - min_bound.x()) * scale_x,
                                   window.getSize().y - (p2.y() - min_bound.y()) * scale_y},
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
static const float MOVE_DIST = 0.01;
static const State MIN_BOUND(-0.1, -0.1);
static const State MAX_BOUND(0.3, 0.1);
static const float SQ_GOAL_TOL = 0.0001;

int main() {
    KDTree<2, 4> kd_tree;

    std::random_device rd;
    std::mt19937 gen(rd());

    auto path_ids = rrt<2>(
        START, GOAL, NUM_POINTS, MOVE_DIST, MIN_BOUND, MAX_BOUND, SQ_GOAL_TOL, kd_tree,
        []([[maybe_unused]] State s) { return true; }, gen);

    run_vis(kd_tree, path_ids, 0, kd_tree.size() - 1, MIN_BOUND, MAX_BOUND);

    return 0;
}
