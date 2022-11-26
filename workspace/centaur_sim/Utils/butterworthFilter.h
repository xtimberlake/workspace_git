/*
 * @Author: haoyun 
 * @Date: 2022-11-23 11:35:59
 * @LastEditors: haoyun 
 * @LastEditTime: 2022-11-23 20:57:52
 * @FilePath: /drake/workspace/centaur_sim/Utils/butterworthFilter.h
 * @Description: implement discrete-time Butterworth filter.
 * 
 *      1. The transfer function is expressed in terms of b and a as
 *         H(z) =  B(z) / A(z)
 *      with B(z) = b[0] + b[1]*z^{-1} + b[2]*z^{-2} + ...
 *      and A(z) = a[0] + a[1]*z^{-1} + a[2]*z^{-2} + ...
 * 
 *      The output is compute by
 *         a[0]*y(k) = (b[0]x(k) + b[1]x(k-1) + b[2]x(k-2) + ...) - (a[1]y(k-1) + a[2]y(k-2) + ...).
 * 
 * 
 *      2. Example: apply a 1-st order discrete time filter
 * 
 *          H(z) = (1 - gamma) / (1 - gamma*z^{-1})
 *      
 *      with gamma = exp(-lambda*T), lambda is the cutoff frequency and T being the sampling time.
 *      
 *      --> initialize:
 *      
 *      
 *      size_t Nth = 1;
 *      DVec<double> num_b = DVec<T>::Zero(Nth + 1);
 *      DVec<double> den_a = DVec<T>::Zero(Nth + 1);
 *      num_b << 1.0-gamma, 0.0;
 *      den_a << 1.0, -gamma;
 *      ButterworthFilter<double> filter(Nth, num_b, den_a);
 * 
 *      --> use:
 *      double x, y;
 *      y = filter.feedData(x);
 * 
 * 
 * 
 * Copyright (c) 2022 by HAR-Lab, All Rights Reserved. 
 */
#pragma once
#include "drake/workspace/centaur_sim/Utils/cppTypes.h"


template <typename T>
class ButterworthFilter
{
public:
	ButterworthFilter(size_t Nth, const DVec<T>& num, const DVec<T>& den) {
		now = 0.0;
		_Nth = Nth;
		_num = num;
		_den = den;
		_coeff = _den[0];
		_flip_num = DVec<T>::Zero(Nth + 1);
		_flip_den_less = DVec<T>::Zero(Nth);
		// std::cout << " _num = " << _num.transpose() << std::endl;
		// std::cout << " _den = " << _den.transpose() << std::endl;


		if (static_cast<size_t>(_num.rows()) != _Nth + 1 || static_cast<size_t>(_den.rows()) != _Nth + 1) {
			throw std::runtime_error("butterworth_filter: the order and the dimension of num/den are not matched! ");
		}
		size_t dim_num = _num.rows();
		for (size_t i = 0; i < dim_num; i++) {
			_flip_num[i] = _num[dim_num - 1 - i];
			if(i < dim_num - 1)
				_flip_den_less[i] = _den[dim_num - 1 - i];
		}
		// std::cout << " _flip_num = " << _flip_num.transpose() << std::endl;
		// std::cout << " _flip_den_less = " << _flip_den_less.transpose() << std::endl;
	}
	~ButterworthFilter() {}
	T feedData(const T x) {
		T res;
		if (_x_vec.size() < _Nth) {
			res = 0.0;
			_x_vec.push_back(x);
			_y_vec.push_back(res);
			
		} else {
			// Step 1) push back new data
			_x_vec.push_back(x);

			// Step 2) calculate the result
			T temp = 0;
			for (size_t j = 0; j < _Nth + 1; j++) {
				temp += _flip_num[j] * _x_vec[j];
			}

			for (size_t k = 0; k < _Nth; k++) {
				temp -= _flip_den_less[k] * _y_vec[k];
			}
			
			res = temp;

			// Step 3) pop the x at the begin
			_x_vec.erase(_x_vec.begin());

			// Step 4) pop & push y vector
			_y_vec.erase(_y_vec.begin());
			_y_vec.push_back(res);
		}

		if(abs(res) > 2000) throw std::runtime_error("the butterworth filter has diverged!");
		now = res;
		return res;
	}


	size_t _Nth; // order
	DVec<T> _num, _flip_num;
	DVec<T> _den, _flip_den_less;
	std::vector<T> _x_vec, _y_vec;
	T _coeff;
	T now;

};

