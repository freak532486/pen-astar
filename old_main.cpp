#include "graph.h"
#include "dijkstra.h"
#include "astar.h"
#include "loader.h"
#include "contraction.h"
#include "potentials.h"
#include "new_potentials.h"
#include "base/vector_io.h"
#include "base/constants.h"
#include "visualisation.h"
#include "penalty.h"
#include "util.h"
#include "performance_logger.h"
#include <string>
#include <iostream>
#include <ctype.h>

const std::string graph_path = "C:/Users/Max/Desktop/graph/germany/";
const std::string contracted_graph_path = "C:/Users/Max/Desktop/graph/germany/travel_time_ch/";
const std::string sources_path = "C:/Users/Max/Desktop/graph/germany/test/source";
const std::string targets_path = "C:/Users/Max/Desktop/graph/germany/test/target";
const std::string node_order_path = "C:/Users/Max/Desktop/graph/germany/travel_time_ch/order";
const std::string test_vector_path = "C:/Users/Max/Desktop/graph/germany/test/travel_time_length";
const std::string latitude_vector_path = "C:/Users/Max/Desktop/graph/germany/latitude";
const std::string longitude_vector_path = "C:/Users/Max/Desktop/graph/germany/longitude";

void test_predefined_order() {
	std::cout << "Reading graph...\n";
	Graph g = read_graph(graph_path);
	std::cout << "|V| = " << g.size() << ", |E| = " << g.n_edges() << "\n";
	std::vector<uint32_t> sources = load_vector<uint32_t>(sources_path);
	std::vector<uint32_t> targets = load_vector<uint32_t>(targets_path);
	std::vector<node_t> order = load_vector<node_t>(node_order_path);
	std::vector<uint32_t> test_values = load_vector<uint32_t>(test_vector_path);
	std::cout << "Contracting graph...\n";
	std::vector<uint32_t> ranking = order_to_ranking(order);
	auto ch = contract_graph(g, order);
	DijkstraService forward_service(ch.forward_graph);
	DijkstraService backward_service(ch.backward_graph);
	std::cout << "Running queries...\n";
	Timer timer;
	for (int i = 0; i < sources.size(); i++) {
		std::cout << "expected dist: " << test_values[i] << ", ";
		timer.lap();
		uint32_t result = dijkstra_on_ch(sources[i], targets[i], forward_service, backward_service, g.size());
		long dt = timer.get();
		std::cout << "dist: " << result << ", dt = " << dt << " mus\n";
	}
}

void test_bottom_up_order() {
	std::cout << "Reading graph...\n";
	Graph g = read_graph(graph_path);
	std::cout << "|V| = " << g.size() << ", |E| = " << g.n_edges() << "\n";
	std::vector<uint32_t> sources = load_vector<uint32_t>(sources_path);
	std::vector<uint32_t> targets = load_vector<uint32_t>(targets_path);
	std::vector<uint32_t> test_values = load_vector<uint32_t>(test_vector_path);
	std::cout << "Contracting graph...\n";
	auto ch = contract_by_queue(g);
	DijkstraService forward_service(ch.forward_graph);
	DijkstraService backward_service(ch.backward_graph);
	std::cout << "Running queries...\n";
	Timer timer;
	for (int i = 0; i < sources.size(); i++) {
		std::cout << "expected dist: " << test_values[i] << ", ";
		timer.lap();
		uint32_t result = dijkstra_on_ch(sources[i], targets[i], forward_service, backward_service, g.size());
		long dt = timer.get();
		std::cout << "dist: " << result << ", dt = " << dt << " mus\n";
	}
}

void test_potentials() {
	std::cout << "Reading graph...\n";
	Graph g = read_graph(graph_path);
	std::cout << "|V| = " << g.size() << ", |E| = " << g.n_edges() << "\n";
	std::vector<uint32_t> sources = load_vector<uint32_t>(sources_path);
	std::vector<uint32_t> targets = load_vector<uint32_t>(targets_path);
	std::vector<node_t> order = load_vector<node_t>(node_order_path);
	std::vector<uint32_t> test_values = load_vector<uint32_t>(test_vector_path);
	std::cout << "Contracting graph...\n";
	std::vector<uint32_t> ranking = order_to_ranking(order);
	Graph g_copy(g);
	auto ch = contract_graph(g_copy, order);
	CHPotentialService potential_service = CHPotentialService(ch);
	AStarService astar_service(g, potential_service);
	std::cout << "Running queries...\n";
	Timer timer;
	for (int i = 0; i < sources.size(); i++) {
		std::cout << "s = " << sources[i] << ", t = " << targets[i] << "\n";
		std::string to_print = "";
		astar_service.add_source(sources[i]);
		potential_service.set_target(targets[i]);
		to_print += "expected dist: " + std::to_string(test_values[i]) + ", ";
		timer.lap();
		astar_service.run_until_target_found(targets[i]);
		uint32_t result = astar_service.get_dist(targets[i]);
		long dt = timer.get();
		to_print += "dist: " + std::to_string(result) + ", dt = " + std::to_string(dt) + "\n";
		astar_service.finish();
		std::cout << to_print;
	}
}

void test_drawing() {
	// Loading vectors
	std::vector<uint32_t> sources = load_vector<uint32_t>(sources_path);
	std::vector<uint32_t> targets = load_vector<uint32_t>(targets_path);
	std::vector<node_t> order = load_vector<node_t>(node_order_path);
	std::vector<uint32_t> ranking = order_to_ranking(order);
	std::vector<float> latitude = load_vector<float>(latitude_vector_path);
	std::vector<float> longitude = load_vector<float>(longitude_vector_path);
	// Contraction and A*-Setup
	Graph g = read_graph(graph_path);
	Graph g_copy(g);
	auto ch = contract_graph(g_copy, order);
	CHPotentialService potential_service(ch);
	AStarService astar_service(g, potential_service);
	// Getting path
	astar_service.add_source(sources[0]);
	astar_service.run_until_target_found(targets[0]);
	Path path = astar_service.get_path(targets[0]);
	// Drawing
	VisualisationService vis_service(g, latitude, longitude, 1000);
	vis_service.clear({ 0, 0, 0 });
	vis_service.draw_graph({ 128, 128, 128 });
	vis_service.draw_path(path, { 255, 0, 0 });
	vis_service.save("test.ppm");
}

void test_alt_with_crx() {
	// Loading vectors
	std::vector<uint32_t> sources = load_vector<uint32_t>(sources_path);
	std::vector<uint32_t> targets = load_vector<uint32_t>(targets_path);
	std::vector<float> latitude = load_vector<float>(latitude_vector_path);
	std::vector<float> longitude = load_vector<float>(longitude_vector_path);
	node_t source = sources[0];
	node_t target = targets[0];
	Graph g = read_graph(graph_path);
	Graph g_copy(g);
	auto ch = read_ch(contracted_graph_path);
	// Find fastest route with A*:
	Timer timer;
	CHPotentialService heur(ch);
	heur.set_target(target);
	AStarService astar(g, heur);
	astar.add_source(source);
	timer.lap();
	astar.run_until_target_found(target);
	Path best_path = astar.get_path(target);
	astar.finish();
	std::cout << "Time for fastest path with dijkstra: " << timer.get() << " microseconds (dist = " << best_path.length << ")\n";
	// Find alternative paths with bidirectional dijkstra
	BidirectionalDijkstraService bi_dij(g, best_path.length * 0.55);
	bi_dij.set_source(source);
	bi_dij.set_target(target);
	bi_dij.run();
	std::vector<Path> alt_paths = bi_dij.get_found_paths();
	bi_dij.finish();
	// Draw
	VisualisationService vis_service(g, latitude, longitude, 1000);
	vis_service.clear({ 0, 0, 0 });
	vis_service.draw_graph({ 128, 128, 128 });
	vis_service.draw_path(best_path, { 255, 0, 0 });
	for (int i = 0; i < alt_paths.size(); i++) {
		vis_service.draw_path(alt_paths[i], { 0, 0, 255 });
	}
	vis_service.save("alt_bi.ppm");
}

void test_penalty() {
	// Loading vectors
	std::vector<uint32_t> sources = load_vector<uint32_t>(sources_path);
	std::vector<uint32_t> targets = load_vector<uint32_t>(targets_path);
	std::vector<float> latitude = load_vector<float>(latitude_vector_path);
	std::vector<float> longitude = load_vector<float>(longitude_vector_path);
	node_t source = sources[0];
	node_t target = targets[0];
	Graph g = read_graph(graph_path);
	Graph g_copy(g);
	auto ch = read_ch(contracted_graph_path);
	// Find fastest route with Dijkstra:
	Timer timer;
	DijkstraService dijkstra(g);
	dijkstra.set_source(source);
	timer.lap();
	dijkstra.run_until_target_found(target);
	uint32_t dist_dijkstra = dijkstra.get_dist(target);
	std::cout << "Time for fastest path with dijkstra: " << timer.get() << " microseconds (dist = " << dist_dijkstra << ")\n";
	// Find fastest route with A-Star
	CHPotentialService heur(ch);
	heur.set_target(target);
	AStarService astar(g, heur);
	astar.add_source(source);
	timer.lap();
	astar.run_until_target_found(target);
	std::cout << "Time for fastest path with a-star and CH-Potentials: " << timer.get() << " microseconds\n";
	auto best_path = astar.get_path(target);
	// Find alternative route
	PenaltyService pen(g, ch);
	pen.set_source(source);
	pen.set_target(target);
	timer.lap();
	pen.run();
	std::cout << "Time for alternative path with penalty-method and CH-Potentials: " << timer.get() << " microseconds\n";
	// Draw everything
	VisualisationService vis_service(g, latitude, longitude, 1000);
	vis_service.clear({ 0, 0, 0 });
	vis_service.draw_graph({ 128, 128, 128 });
	vis_service.draw_subgraph(pen.get_alt_graph(), { 255, 0, 255 });
	vis_service.draw_path(best_path, { 255, 0, 0 });
	vis_service.save("pen.ppm");
}

void test_penalty_dijkstra_rank() {
	Graph g = read_graph(graph_path);
	ContractionHierarchy ch = read_ch(contracted_graph_path);
	DijkstraService forward_service(ch.forward_graph);
	DijkstraService backward_service(ch.backward_graph);
	std::vector<node_t> sources = load_vector<node_t>(sources_path);
	PenaltyService pen(g, ch);
	Timer timer;
	ProgressBar pbar;
	int n_iterations = 10;
	for (int i = 0; i < 10; i++) {
		std::vector<node_t> targets = get_dijkstra_rank_nodes(g, sources[i]);
		for (int j = 0; j < targets.size(); j++) {
			global_performance_logger.begin_test_case();
			global_performance_logger.set_source(sources[i]);
			global_performance_logger.set_target(targets[j]);
			global_performance_logger.set_dijkstra_rank(j);
			global_performance_logger.log_shortest_path_length(dijkstra_on_ch(sources[i], targets[j], forward_service, backward_service, g.size()));
			pen.set_source(sources[i]);
			pen.set_target(targets[j]);
			timer.lap();
			pen.run();
			global_performance_logger.log_total_runtime(timer.get());
			pen.reset();
			pbar.update_progress((double)i / n_iterations + (double)j / (n_iterations * targets.size()));
		}
	}
	pbar.finish();
	std::cout << global_performance_logger.results_to_json_string();
}

/* int main(int argc, const char** argv) {
	test_penalty_dijkstra_rank();
} */