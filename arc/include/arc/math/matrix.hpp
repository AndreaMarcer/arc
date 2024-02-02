/**
 * @file constants.hpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <type_traits>
#include <initializer_list>

#include "pico/stdlib.h"
#include "common/log.hpp"

namespace arc {
namespace math {

template <typename T, uint8_t ROWS, uint8_t COLS>
class Matrix {
    public:
	static constexpr uint8_t m_cols = ROWS;
	static constexpr uint8_t m_rows = COLS;

	Matrix(); // Default
	Matrix(const T &); // Default
	Matrix(const T (&)[ROWS][COLS]); // Default
	Matrix(const std::initializer_list<std::initializer_list<T> > &);
	Matrix(const Matrix &); // copy
	Matrix(Matrix &&); // move

	~Matrix();

	constexpr T *operator[](uint8_t n) const { return m_matrix + n * COLS; }
	Matrix operator*(const T &) const;
	Matrix operator*(const T &&) const;

	Matrix operator+(const Matrix &) const;
	Matrix operator-(const Matrix &) const;

	constexpr Matrix &operator+=(const Matrix &);
	constexpr Matrix &operator-=(const Matrix &);
	constexpr Matrix &operator*=(const T &);
	constexpr Matrix &operator*=(const T &&);

	constexpr Matrix &operator=(const Matrix &);
	constexpr Matrix &operator=(Matrix &&);

	constexpr Matrix &clear();
	constexpr Matrix &fill(const T &);
	void print() const;

    private:
	void print(const char[]) const;

	T *m_matrix = new T[ROWS * COLS];
};

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix() // Default
{
	log_info("MATRIX Default Constructor\n");
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix(const T &v) // Default
{
	log_info("MATRIX Default Constructor with fill value\n");
	fill(v);
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix(const T (&matrix)[ROWS][COLS]) // Default
{
	log_info("MATRIX Constructor bidimensional array\n");
	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m_matrix[r * COLS + c] = matrix[r][c];
		}
	}
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix(
	const std::initializer_list<std::initializer_list<T> > &matrix)
{
	log_info("MATRIX Constructor initializer_list\n");

	auto it_r = matrix.begin();
	for (uint8_t r = 0; r < ROWS && it_r != matrix.end(); ++r, ++it_r) {
		auto it_c = it_r->begin();
		for (uint8_t c = 0; c < COLS && it_c != it_r->end();
		     ++c, ++it_c) {
			m_matrix[r * COLS + c] = *it_c;
		}
	}
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix(const Matrix &other) // Copy
{
	log_info("MATRIX Constructor COPY\n");
	memcpy(m_matrix, other.m_matrix, COLS * ROWS * sizeof(T));
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix(Matrix &&other) // Move
{
	log_info("MATRIX Constructor MOVE\n");
	delete[] m_matrix;
	m_matrix = other.m_matrix;
	other.m_matrix = nullptr;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::~Matrix()
{
	log_info("MATRIX Destructor\n");
	delete[] m_matrix;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS> Matrix<T, ROWS, COLS>::operator*(const T &s) const
{
	log_info("MATRIX MOLT &matrix x &scalar\n");

	Matrix m;
	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m[r][c] = m_matrix[r * COLS + c] * s;
		}
	}
	return m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS> Matrix<T, ROWS, COLS>::operator*(const T &&s) const
{
	log_info("MATRIX MOLT matrix x &&scalar\n");

	Matrix m;
	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m[r][c] = m_matrix[r * COLS + c] * s;
		}
	}
	return m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS> Matrix<T, ROWS, COLS>::operator+(const Matrix &r_m) const
{
	log_info("MATRIX ADD &matrix + &matrix\n");

	Matrix m;
	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m[r][c] = m_matrix[r * COLS + c] + r_m[r][c];
		}
	}
	return m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS> Matrix<T, ROWS, COLS>::operator-(const Matrix &r_m) const
{
	log_info("MATRIX SUB &matrix - &matrix\n");

	Matrix m;
	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m[r][c] = m_matrix[r * COLS + c] - r_m[r][c];
		}
	}
	return m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
constexpr Matrix<T, ROWS, COLS> &
Matrix<T, ROWS, COLS>::operator+=(const Matrix &r_m)
{
	log_info("MATRIX ADD this += &matrix\n");

	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m_matrix[r * COLS + c] += r_m[r][c];
		}
	}
	return *this;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
constexpr Matrix<T, ROWS, COLS> &
Matrix<T, ROWS, COLS>::operator-=(const Matrix &r_m)
{
	log_info("MATRIX SUB this -= &matrix\n");

	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m_matrix[r * COLS + c] -= r_m[r][c];
		}
	}
	return *this;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
constexpr Matrix<T, ROWS, COLS> &Matrix<T, ROWS, COLS>::operator*=(const T &s)
{
	log_info("MATRIX Molt this *= &scalar\n");

	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m_matrix[r * COLS + c] *= s;
		}
	}
	return *this;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
constexpr Matrix<T, ROWS, COLS> &Matrix<T, ROWS, COLS>::operator*=(const T &&s)
{
	log_info("MATRIX Molt this *= &&scalar\n");

	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m_matrix[r * COLS + c] *= s;
		}
	}
	return *this;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
constexpr Matrix<T, ROWS, COLS> &
Matrix<T, ROWS, COLS>::operator=(const Matrix<T, ROWS, COLS> &other)
{
	log_info("MATRIX ASSIGN reference => copy\n");
	if (this != &other) {
		memcpy(m_matrix, &other, COLS * ROWS * sizeof(T));
	}
	return *this;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
constexpr Matrix<T, ROWS, COLS> &
Matrix<T, ROWS, COLS>::operator=(Matrix<T, ROWS, COLS> &&other)
{
	log_info("MATRIX ASSIGN r_value => move\n");
	delete[] m_matrix;
	m_matrix = other.m_matrix;
	other.m_matrix = nullptr;
	return *this;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
constexpr Matrix<T, ROWS, COLS> &Matrix<T, ROWS, COLS>::clear()
{
	memset(m_matrix, 0, COLS * ROWS * sizeof(T));
	return *this;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
constexpr Matrix<T, ROWS, COLS> &Matrix<T, ROWS, COLS>::fill(const T &v)
{
	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m_matrix[r * COLS + c] = v;
		}
	}
	return *this;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
void Matrix<T, ROWS, COLS>::print() const
{
	if constexpr (std::is_integral_v<T>) {
		if constexpr (std::is_unsigned_v<T>) {
			print("%u, ");
		} else {
			print("%d, ");
		}
	} else {
		if (std::is_floating_point_v<T>) {
			print("%f, ");
		} else {
			return;
		}
	}
}

//=============================================================================
//						        PRIVATE METHODS
//=============================================================================

template <typename T, uint8_t ROWS, uint8_t COLS>
void Matrix<T, ROWS, COLS>::print(const char fmt[]) const
{
	for (uint8_t r = 0; r < ROWS; r++) {
		log_debug("");
		for (uint8_t c = 0; c < COLS; c++) {
			log_debug_s(fmt, m_matrix[c * COLS + r]);
		}
		log_debug_s("\n");
	}
}

//=============================================================================
//						        OTHER OPERATORS
//=============================================================================

template <typename T, uint8_t ROWS, uint8_t COLS>
inline const Matrix<T, ROWS, COLS> operator*(const T &s,
					     const Matrix<T, ROWS, COLS> &r_m)
{
	log_info("MATRIX MOLT &scalar x &matrix\n");

	Matrix<T, ROWS, COLS> m;
	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m[r][c] = r_m[r][c] * s;
		}
	}
	return m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
inline const Matrix<T, ROWS, COLS> operator*(const T &&s,
					     const Matrix<T, ROWS, COLS> r_m)
{
	log_info("MATRIX MOLT &&scalar x &matrix\n");

	Matrix<T, ROWS, COLS> m;
	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			m[r][c] = r_m[r][c] * s;
		}
	}
	return m;
}

template <typename T, uint8_t L_ROWS, uint8_t L_COLS_R_ROWS, uint8_t R_COLS>
inline const Matrix<T, L_ROWS, R_COLS>
operator*(const Matrix<T, L_ROWS, L_COLS_R_ROWS> &l_m,
	  const Matrix<T, L_COLS_R_ROWS, R_COLS> &r_m)
{
	log_info("MATRIX MOLT &[%hhu,%hhu] x &[%hhu,%hhu] \n", L_ROWS,
		 L_COLS_R_ROWS, L_COLS_R_ROWS, R_COLS);

	Matrix<T, L_ROWS, R_COLS> m;
	m.clear();
	for (uint8_t l_r = 0; l_r < L_ROWS; l_r++) {
		for (uint8_t r_c = 0; r_c < R_COLS; r_c++) {
			for (uint8_t i = 0; i < L_COLS_R_ROWS; i++) {
				m[l_r][r_c] += l_m[l_r][i] * r_m[i][r_c];
			}
		}
	}
	return m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
inline const Matrix<T, ROWS, COLS> &operator*=(Matrix<T, ROWS, COLS> &l_m,
					       const Matrix<T, ROWS, COLS> &r_m)
{
	log_info("MATRIX &matrix *= &matrix\n");
	return l_m = l_m * r_m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
inline const bool operator!=(const Matrix<T, ROWS, COLS> &l_m,
			     const Matrix<T, ROWS, COLS> &r_m)
{
	log_info("MATRIX this != &matrix\n");

	if (&l_m == &r_m)
		return false;

	for (uint8_t r = 0; r < ROWS; r++) {
		for (uint8_t c = 0; c < COLS; c++) {
			if (l_m[r][c] != r_m[r][c])
				return true;
		}
	}
	return false;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
inline const bool operator==(const Matrix<T, ROWS, COLS> &l_m,
			     const Matrix<T, ROWS, COLS> &r_m)
{
	log_info("MATRIX this == &matrix\n");

	return !(l_m != r_m);
}

} // arc
} // math