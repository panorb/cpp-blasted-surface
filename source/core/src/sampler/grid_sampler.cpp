#include "blast/sampler/grid_sampler.hpp"

#include <imgui.h>
#include <stack>

using Image = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

void Grid_sampler::render_controls()
{
	ImGui::DragFloat("Grid size", &grid_size_, .5, 1.0, 30.0);
	if (ImGui::BeginTable("table", 2))
	{
		ImGui::TableNextColumn();
		ImGui::DragInt("Circle radius", &circle_radius_, 1, 1, 30);
		ImGui::TableNextColumn();
		ImGui::DragInt("Erosion iterations", &erosion_iterations_, 1, 0, 80);
		ImGui::EndTable();
	}
	ImGui::Checkbox("Fill holes", &hole_filling_enabled_);
	

	// circle_radius, erosion_iterations, hole_filling_enabled
}

void output_image(const Image& image)
{
	std::ostringstream oss;

	// Iterate through the rows and columns of the image
	for (int row = 0; row < image.rows(); ++row) {
		for (int col = 0; col < image.cols(); ++col) {
			// Append each value, formatting as needed
			oss << (image(row, col) > 0 ? "@" : ".") << " "; // Use '@' for non-zero, '.' for zero
		}
		oss << '\n'; // New line at the end of each row
	}

	// Output the formatted string to spdlog
	spdlog::info("Image:\n{}", oss.str());
}

void erode_image(Image& image, size_t iterations)
{
	// Temporary copy of the image to work on
	Image temp = image;

	// Get the image dimensions
	int rows = image.rows();
	int cols = image.cols();

	// Define the 8-connected neighborhood offsets
	std::vector<Eigen::Vector2i> neighbors = {
		{-1, -1}, {-1, 0}, {-1, 1},
		{0, -1},          {0, 1},
		{1, -1}, {1, 0}, {1, 1} };

	for (size_t iter = 0; iter < iterations; ++iter) {
		// Process each pixel
		for (int row = 0; row < rows; ++row) {
			for (int col = 0; col < cols; ++col) {
				// Check if the pixel is at the border of the image
				if (row == 0 || row == rows - 1 || col == 0 || col == cols - 1) {
					// Erode border pixels directly
					temp(row, col) = 0.0;
					continue;
				}

				// Check neighbors for non-border pixels
				bool should_erode = false;
				for (const auto& offset : neighbors) {
					int neighbor_row = row + offset.x();
					int neighbor_col = col + offset.y();

					// If the neighbor is within bounds and is zero
					if (neighbor_row >= 0 && neighbor_row < rows &&
						neighbor_col >= 0 && neighbor_col < cols &&
						image(neighbor_row, neighbor_col) == 0.0) {
						should_erode = true;
						break;
					}
				}

				// Mark the pixel for erosion
				if (should_erode) {
					temp(row, col) = 0.0;
				}
			}
		}

		// Update the original image for the next iteration
		image = temp;
	}
}

void fill_holes(Image& image)
{
	// Get the dimensions of the image
	int rows = image.rows();
	int cols = image.cols();

	// Create a temporary matrix to track visited pixels
	Image visited = Image::Zero(rows, cols);

	// Lambda function for flood-fill
	auto flood_fill = [&](int start_row, int start_col, float old_value, float new_value) {
		std::stack<Eigen::Vector2i> stack;
		stack.push(Eigen::Vector2i(start_row, start_col));

		while (!stack.empty()) {
			Eigen::Vector2i pixel = stack.top();
			stack.pop();

			int r = pixel.x();
			int c = pixel.y();

			// Skip if out of bounds or already visited
			if (r < 0 || r >= rows || c < 0 || c >= cols || visited(r, c) == 1 || image(r, c) != old_value) {
				continue;
			}

			// Mark as visited and set the new value
			visited(r, c) = 1;
			image(r, c) = new_value;

			// Push neighbors onto the stack
			stack.push(Eigen::Vector2i(r - 1, c)); // Up
			stack.push(Eigen::Vector2i(r + 1, c)); // Down
			stack.push(Eigen::Vector2i(r, c - 1)); // Left
			stack.push(Eigen::Vector2i(r, c + 1)); // Right
		}
		};

	// Step 1: Flood-fill from the border to mark non-hole regions
	for (int col = 0; col < cols; ++col) {
		if (image(0, col) == 0) flood_fill(0, col, 0, -1);            // Top border
		if (image(rows - 1, col) == 0) flood_fill(rows - 1, col, 0, -1); // Bottom border
	}
	for (int row = 0; row < rows; ++row) {
		if (image(row, 0) == 0) flood_fill(row, 0, 0, -1);            // Left border
		if (image(row, cols - 1) == 0) flood_fill(row, cols - 1, 0, -1); // Right border
	}

	// Step 2: Fill all remaining zeros (holes) with ones
	for (int row = 0; row < rows; ++row) {
		for (int col = 0; col < cols; ++col) {
			if (image(row, col) == 0) {
				image(row, col) = 1; // Fill the hole
			}
		}
	}

	// Step 3: Restore the border-marked regions back to zeros
	for (int row = 0; row < rows; ++row) {
		for (int col = 0; col < cols; ++col) {
			if (image(row, col) == -1) {
				image(row, col) = 0; // Restore the border-connected regions
			}
		}
	}
}


void draw_circle(Image& image, Eigen::Vector2i image_point, int radius)
{
	// Get the bounds of the image
	int rows = image.rows();
	int cols = image.cols();

	// Iterate through a bounding box around the circle
	for (int y = -radius; y <= radius; ++y) {
		for (int x = -radius; x <= radius; ++x) {
			// Compute the squared distance from the center
			int dist_sq = x * x + y * y;

			// Check if the point lies within the circle's radius
			if (dist_sq <= radius * radius) {
				// Compute the image coordinates
				int px = image_point.x() + x;
				int py = image_point.y() + y;

				// Check bounds and set the pixel value if valid
				if (px >= 0 && px < cols && py >= 0 && py < rows) {
					image(py, px) = 1.0; // Assuming the image values are float/double
				}
			}
		}
	}
}

std::vector<Eigen::Vector3f> Grid_sampler::sample(Oriented_bounding_box& bbox, const std::vector<Eigen::Vector3f>& cloud_points)
{
    Image image;
    image.resize(static_cast<int>(bbox.extent_(0) / 0.1), static_cast<int>(bbox.extent_(1) / 0.1));
	image.setZero();

    // Calculate the offset from the center to the upper left corner
    Eigen::Vector3f upper_left_corner = bbox.get_center() - 0.5 * bbox.extent_(0) * bbox.R_.col(0) - 0.5 * bbox.extent_(1) * bbox.R_.col(1);

    for (auto& pt : cloud_points)
    {
        Eigen::Vector3f local_point = bbox.R_.transpose() * (pt - upper_left_corner);
        Eigen::Vector2f local_point_2d = local_point.head<2>();
        Eigen::Vector2f local_point_2d_normalized = local_point_2d.cwiseQuotient(bbox.extent_.head<2>());
        Eigen::Vector2i image_point = (local_point_2d_normalized * image.cols()).cast<int>();

		if (image_point(0) < 0 || image_point(0) >= image.cols() || image_point(1) < 0 || image_point(1) >= image.rows())
		{
			continue;
		}

		// Draw circle with radius onto image
		draw_circle(image, image_point, circle_radius_);

		// image(image_point(1), image_point(0)) = 1.0;
		// image(image_point(1), image_point(0)) = 1.0;
    }

	if (hole_filling_enabled_)
	{
		fill_holes(image);
	}

	if (erosion_iterations_ > 0)
	{
		erode_image(image, erosion_iterations_);
	}

	output_image(image);
	

	Eigen::Vector3f normal = (bbox.R_* Eigen::Vector3f(0, 0, 1)).normalized();

	normal.normalize();

	Eigen::Vector3f x_axis = bbox.R_ * Eigen::Vector3f(1, 0, 0);
	Eigen::Vector3f y_axis = bbox.R_ * Eigen::Vector3f(0, 1, 0);

	Eigen::Vector3f origin = bbox.get_center() - 0.5 * bbox.extent_(0) * x_axis - 0.5 * bbox.extent_(1) * y_axis;

	Eigen::Vector3f x_axis_unit = x_axis.normalized();
	Eigen::Vector3f y_axis_unit = y_axis.normalized();

	float x_axis_length = bbox.extent_(0);
	float y_axis_length = bbox.extent_(1);

	int x_axis_num = x_axis_length / grid_size_;
	int y_axis_num = y_axis_length / grid_size_;


	std::vector<Eigen::Vector3f> sample_points;

	for (int i = 1; i < x_axis_num - 1; i++)
	{
		for (int j = 1; j < y_axis_num - 1; j++)
		{
			Eigen::Vector3f point = origin + i * grid_size_ * x_axis_unit + j * grid_size_ * y_axis_unit;

			// check if point is supported by image
			Eigen::Vector3f local_point = bbox.R_.transpose() * (point - upper_left_corner);
			Eigen::Vector2f local_point_2d = local_point.head<2>();
			Eigen::Vector2f local_point_2d_normalized = local_point_2d.cwiseQuotient(bbox.extent_.head<2>());
			Eigen::Vector2i image_point = (local_point_2d_normalized * image.cols()).cast<int>();

			if (image_point(0) < 0 || image_point(0) >= image.cols() || image_point(1) < 0 || image_point(1) >= image.rows())
			{
				continue;
			}

			if (image(image_point(1), image_point(0)) == 1.0)
			{
				sample_points.push_back(point);
			}
		}
	}

	return sample_points;
}
