#pragma once

#include "graph.h"
#include <spatium/Image.h>
#include <spatium/ImageIO.h>
#include <spatium/gfx2d/Drawing.h>
#include <vector>

// Note: Latitude is y-coord, longitude is x-coord.
// Geographers hate me

struct Color {
	uint8_t r;
	uint8_t g;
	uint8_t b;

	std::array<uint8_t, 3> _array() {
		return { r, g, b };
	}
};

float find_min(const std::vector<float>& vec) {
	float min = INFINITY;
	for (auto i = vec.begin(); i < vec.end(); i++) {
		if (*i < min) {
			min = *i;
		}
	}
	return min;
}

float find_max(const std::vector<float>& vec) {
	float max = -INFINITY;
	for (auto i = vec.begin(); i < vec.end(); i++) {
		if (*i > max) {
			max = *i;
		}
	}
	return max;
}

float get_aspect_ratio(const std::vector<float>& lat, const std::vector<float>& lng) {
	float min_long = find_min(lng);
	float min_lat = find_min(lng);
	float max_long = find_max(lat);
	float max_lat = find_max(lat);
	return (max_long - min_long) / (max_lat - min_lat);
}

class VisualisationService {

private:
	const Graph& g;
	std::vector<float>& lat;
	std::vector<float>& lng;
	spatium::Image<> image;
	uint32_t width, height;
	float min_lat, max_lat, min_lng, max_lng;

	int get_x_coord(float lng) {
		return std::floor(width * (lng - min_lng) / (max_lng - min_lng));
	}

	int get_y_coord(float lat) {
		return std::floor(height * (1 - (lat - min_lat) / (max_lat - min_lat)));
	}


public:

	VisualisationService(const Graph& g, std::vector<float>& lat, std::vector<float>& lng, int image_height) : g(g), lat(lat), lng(lng), image(std::floor(get_aspect_ratio(lat, lng) * image_height), image_height) {
		width = image.width();
		height = image.height();
		min_lat = find_min(lat);
		max_lat = find_max(lat);
		min_lng = find_min(lng);
		max_lng = find_max(lng);
	}

	void clear(Color c) {
		spatium::gfx2d::Drawing::drawRectangle<uint8_t, 3>(image, { 0, 0 }, { (int)width, (int)height }, c._array(), true);
	}

	void draw_edge(node_t a, node_t b, Color c) {
		int ax = get_x_coord(lng[a]);
		int ay = get_y_coord(lat[a]);
		int bx = get_x_coord(lng[b]);
		int by = get_y_coord(lat[b]);
		spatium::gfx2d::Drawing::drawLine<uint8_t, 3>(image, { ax, ay }, { bx, by }, c._array());
	}

	void draw_graph(Color c) {
		for (node_t n = 0; n < g.size(); n++) {
			const std::vector<Edge>& arcs = g.get_out_arcs(n);
			for (auto j = arcs.begin(); j < arcs.end(); j++) {
				draw_edge(n, j->target, c);
			}
		}
	}

	void draw_subgraph(const Graph& g, Color c) {
		for (node_t n = 0; n < g.size(); n++) {
			const std::vector<Edge>& arcs = g.get_out_arcs(n);
			for (auto j = arcs.begin(); j < arcs.end(); j++) {
				draw_edge(n, j->target, c);
			}
		}
	}

	void draw_path(const Path& path, Color c) {
		for (auto i = path.nodes.begin(); i < path.nodes.end() - 1; i++) {
			int x = get_x_coord(lng[*i]);
			int _x = get_x_coord(lng[*(i + 1)]);
			int y = get_y_coord(lat[*i]);
			int _y = get_y_coord(lat[*(i + 1)]);
			spatium::gfx2d::Drawing::drawLine<uint8_t, 3>(image, { x, y }, { _x, _y }, c._array());
		}
	}

	bool save(std::string path) {
		return spatium::ImageIO::writeRgbImageAsPpm(image, path);
	}

};
