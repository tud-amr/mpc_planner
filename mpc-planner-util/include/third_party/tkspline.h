/*
 * spline.h
 *
 * simple cubic spline interpolation library without external
 * dependencies
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2011, 2014 Tino Kluge (ttk448 at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 */

#ifndef TK_SPLINE_H
#define TK_SPLINE_H

#include <cstdio>
#include <cassert>
#include <vector>
#include <algorithm>

// unnamed namespace only because the implementation is in this
// header file and we don't want to export symbols to the obj files

namespace tk
{

	// band matrix solver
	class band_matrix
	{
	private:
		std::vector<std::vector<double>> m_upper; // upper band
		std::vector<std::vector<double>> m_lower; // lower band
	public:
		band_matrix(){};						// constructor
		band_matrix(int dim, int n_u, int n_l); // constructor
		~band_matrix(){};						// destructor
		void resize(int dim, int n_u, int n_l); // init with dim,n_u,n_l
		int dim() const;						// matrix dimension
		int num_upper() const
		{
			return m_upper.size() - 1;
		}
		int num_lower() const
		{
			return m_lower.size() - 1;
		}
		// access operator
		double &operator()(int i, int j);	   // write
		double operator()(int i, int j) const; // read
		// we can store an additional diogonal (in m_lower)
		double &saved_diag(int i);
		double saved_diag(int i) const;
		void lu_decompose();
		std::vector<double> r_solve(const std::vector<double> &b) const;
		std::vector<double> l_solve(const std::vector<double> &b) const;
		std::vector<double> lu_solve(const std::vector<double> &b,
									 bool is_lu_decomposed = false);
	};

	// spline interpolation
	class spline
	{
	public:
		enum bd_type
		{
			first_deriv = 1,
			second_deriv = 2
		};

	private:
		std::vector<double> m_x, m_y; // x,y coordinates of points
		// interpolation parameters
		// f(x) = a*(x-x_i)^3 + b*(x-x_i)^2 + c*(x-x_i) + y_i

		double m_b0, m_c0; // for left extrapol
		bd_type m_left, m_right;
		double m_left_value, m_right_value;
		bool m_force_linear_extrapolation;

	public:
		std::vector<double> m_a, m_b, m_c, m_d; // spline coefficients
		// set default boundary condition to be zero curvature at both ends
		std::vector<double> m_x_, m_y_; // x,y coordinates of points
		spline() : m_left(second_deriv), m_right(second_deriv),
				   m_left_value(0.0), m_right_value(0.0),
				   m_force_linear_extrapolation(false) {}

		// optional, but if called it has to come be before set_points()
		void set_boundary(bd_type left, double left_value,
						  bd_type right, double right_value,
						  bool force_linear_extrapolation = false);
		void set_points(const std::vector<double> &x,
						const std::vector<double> &y, bool cubic_spline = true);
		double operator()(double x) const;
		double deriv(int order, double x) const;

		// void removeStart()
		// {
		// 	// Remove the first element of all computed / input vectors
		// 	m_a.erase(m_a.begin());
		// 	m_b.erase(m_b.begin());
		// 	m_c.erase(m_c.begin());
		// 	m_d.erase(m_d.begin());

		// 	m_x.erase(m_x.begin());
		// 	m_y.erase(m_y.begin());

		// 	m_x_.erase(m_x_.begin());
		// 	m_y_.erase(m_y_.begin());
		// }

		/** @brief Add interface for retrieving a, b, c, d */
		void getParameters(int index, double &a, double &b, double &c, double &d) const
		{
			assert(index <= (int)m_a.size() - 1);
			a = m_a[index];
			b = m_b[index];
			c = m_c[index];
			d = m_d[index];
		}

		double getSplineStart(int index)
		{
			assert(index >= 0);					 // N - 1 segments
			assert(index < (int)m_x.size() - 1); // N - 1 segments
			return m_x[index];
		}
		double getSplineEnd(int index)
		{
			assert(index >= 0);					 // N - 1 segments
			assert(index < (int)m_x.size() - 1); // N - 1 segments
			return m_x[index + 1];
		}

		// void Print()
		// {
		// 	std::cout << "=== tkspline ===\n";
		// 	std::cout << "Segments (m_a size) = " << m_a.size() << std::endl;
		// 	std::cout << "Points (m_x size) = " << m_x.size() << std::endl;
		// 	std::cout << "Points:\n";
		// 	for (size_t i = 0; i < m_x.size(); i++)
		// 	{
		// 		std::cout << "(" << m_x[i] << ", " << m_y[i] << ")\n";
		// 	}
		// 	std::cout << "\nSplines:\n";
		// 	for (size_t i = 0; i < m_a.size(); i++)
		// 	{
		// 		std::cout << "a = " << m_a[i] << ",\tb = " << m_b[i] << ",\tc = " << m_c[i] << ",\td = " << m_d[i] << std::endl;
		// 	}
		// 	std::cout << "================\n";
		// }
	};

} // namespace tk

#endif /* TK_SPLINE_H */
