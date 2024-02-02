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

	void print();

	constexpr T *operator[](uint8_t n)
	{
		return m_matrix + n * COLS;
	}

	Matrix operator*(Matrix &);
	Matrix operator*(T &&);
	Matrix operator*(T &);

	constexpr Matrix &operator=(const Matrix &);
	constexpr Matrix &operator=(Matrix &&);

	void clear();
	void fill(const T &);

    private:
	void print(const char[]);

	T *m_matrix = new T[ROWS * COLS];
};

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix() // Default
{
	log_info("MATRIX Default\n");
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix(const T &v) // Default
{
	log_info("MATRIX Default with value\n");
	fill(v);
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix(const Matrix &other) // copy
{
	log_info("MATRIX copy\n");
	memcpy(m_matrix, other.m_matrix, COLS * ROWS * sizeof(T));
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix(Matrix &&other)
	: m_matrix{ other.m_matrix } // move
{
	log_info("MATRIX move\n");
	other.m_matrix = nullptr;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix(
	const std::initializer_list<std::initializer_list<T> > &matrix)
{
	log_info("MATRIX initializer_list\n");

	auto it_r = matrix.begin();
	for (uint8_t i = 0; i < ROWS && it_r != matrix.end(); ++i, ++it_r) {
		auto it_c = it_r->begin();
		for (uint8_t j = 0; j < COLS && it_c != it_r->end();
		     ++j, ++it_c) {
			m_matrix[i * COLS + j] = *it_c;
		}
	}
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::Matrix(const T (&matrix)[ROWS][COLS]) // Default
{
	log_info("MATRIX bidimensional array\n");
	for (uint8_t i = 0; i < ROWS; i++) {
		for (uint8_t j = 0; j < COLS; j++) {
			m_matrix[i * COLS + j] = matrix[i][j];
		}
	}
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS>::~Matrix()
{
	log_info("MATRIX Default Destructor\n");
	delete[] m_matrix;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS> Matrix<T, ROWS, COLS>::operator*(Matrix &r_m)
{
	log_info("MATRIX moltiplication by matrix\n");

	Matrix m;
	for (uint8_t i = 0; i < ROWS; i++) {
		for (uint8_t j = 0; j < COLS; j++) {
			m[i][j] = m_matrix[i * COLS + j] * r_m[i][j];
		}
	}
	return m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS> Matrix<T, ROWS, COLS>::operator*(T &&s)
{
	log_info("MATRIX moltiplication by r_value scalar\n");

	Matrix m;
	for (uint8_t i = 0; i < ROWS; i++) {
		for (uint8_t j = 0; j < COLS; j++) {
			m[i][j] = m_matrix[i * COLS + j] * s;
		}
	}
	return m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
Matrix<T, ROWS, COLS> Matrix<T, ROWS, COLS>::operator*(T &s)
{
	log_info("MATRIX moltiplication by reference scalar\n");

	Matrix m;
	for (uint8_t i = 0; i < ROWS; i++) {
		for (uint8_t j = 0; j < COLS; j++) {
			m[i][j] = m_matrix[i * COLS + j] * s;
		}
	}
	return m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
constexpr Matrix<T, ROWS, COLS> &
Matrix<T, ROWS, COLS>::operator=(const Matrix<T, ROWS, COLS> &matrix)
{
	log_info("MATRIX assign reference\n");
	if (this != &matrix) {
		memcpy(m_matrix, &matrix, COLS * ROWS * sizeof(T));
	}
	return *this;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
constexpr Matrix<T, ROWS, COLS> &
Matrix<T, ROWS, COLS>::operator=(Matrix<T, ROWS, COLS> &&other)
{
	log_info("MATRIX assign r_value\n");
	m_matrix = other.m_matrix;
	delete[] m_matrix;
	other.m_matrix = nullptr;
	return *this;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
void Matrix<T, ROWS, COLS>::clear()
{
	memset(m_matrix, 0, COLS * ROWS * sizeof(T));
}

template <typename T, uint8_t ROWS, uint8_t COLS>
void Matrix<T, ROWS, COLS>::fill(const T &v)
{
	for (uint8_t i = 0; i < ROWS; i++) {
		for (uint8_t j = 0; j < COLS; j++) {
			m_matrix[i * COLS + j] = v;
		}
	}
}

template <typename T, uint8_t L_ROWS, uint8_t L_COLS_R_ROWS, uint8_t R_COLS>
Matrix<T, L_ROWS, R_COLS> operator*(Matrix<T, L_ROWS, L_COLS_R_ROWS> l_m,
				    Matrix<T, L_COLS_R_ROWS, R_COLS> r_m)
{
	log_info("MATRIX moltiplication [%hhu,%hhu] * [%hhu,%hhu] \n", L_ROWS,
		 L_COLS_R_ROWS, L_COLS_R_ROWS, R_COLS);

	Matrix<T, L_ROWS, R_COLS> m;
	m.clear();
	for (uint8_t i = 0; i < L_ROWS; i++) {
		for (uint8_t j = 0; j < R_COLS; j++) {
			for (uint8_t k = 0; k < L_COLS_R_ROWS; k++) {
				m[i][j] += l_m[i][k] * r_m[k][j];
			}
		}
	}
	return m;
}

template <typename T, uint8_t ROWS, uint8_t COLS>
void Matrix<T, ROWS, COLS>::print(const char fmt[])
{
	for (uint8_t i = 0; i < ROWS; i++) {
		log_debug("");
		for (uint8_t j = 0; j < COLS; j++) {
			log_debug_s(fmt, m_matrix[i * COLS + j]);
		}
		log_debug_s("\n");
	}
}

template <typename T, uint8_t ROWS, uint8_t COLS>
void Matrix<T, ROWS, COLS>::print()
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

} // arc
} // math