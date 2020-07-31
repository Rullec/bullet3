#include "tools.h"
// #include <emmintrin.h>
#include <iostream>
#include <immintrin.h>
tMatrix Tools::t = tMatrix::Zero();
tMatrix Tools::t1 = tMatrix::Zero();
tMatrix Tools::t2 = tMatrix::Zero();
void Tools::AVX4x4v1(const tMatrix& a, const tMatrix& b, tMatrix& r)
{
	r.noalias() = a * b;
}

void Tools::AVX4x4v1_3mat(const tMatrix& a, const tMatrix& b, const tMatrix& c, tMatrix& r)
{
	r.noalias() = a * b * c;
}

// void Tools::Multiply3x3(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r) {
// 	//// brute force
// 	//for (int i = 0; i < 3; ++i) {
// 	//	for (int j = 0; j < 3; ++j) {
// 	//		double sum = 0;
// 	//		for(int k = 0; k < 3; ++k) {
// 	//			sum += a.coeffRef(i, k) * b.coeffRef(k, j);
// 	//		}
// 	//		r.coeffRef(i, j) = sum;
// 	//	}
// 	//}

// 	// cache friendly
// 	//for (int i = 0; i < 3; ++i) {
// 	//	for (int k = 0; k < 3; ++k) {
// 	//		auto& t = a.coeffRef(i, k);
// 	//		for(int j = 0; j < 3; ++j) {
// 	//			r.coeffRef(i, j) += t * b.coeffRef(k, j);
// 	//		}
// 	//	}
// 	//}

// 	// loop unrolling 1
// 	//for (int i = 0; i < 3; ++i) {
// 	//	for (int k = 0; k < 3; ++k) {
// 	//		auto& t = a.coeffRef(i, k);
// 	//		r.coeffRef(i, 0) += t * b.coeffRef(k, 0);
// 	//		r.coeffRef(i, 1) += t * b.coeffRef(k, 1);
// 	//		r.coeffRef(i, 2) += t * b.coeffRef(k, 2);
// 	//	}
// 	//}

// 	//// loop unrolling 2
// 	//for (int i = 0; i < 3; ++i) {
// 	//	r.coeffRef(i, 0) += a.coeffRef(i, 0) * b.coeffRef(0, 0);
// 	//	r.coeffRef(i, 1) += a.coeffRef(i, 0) * b.coeffRef(0, 1);
// 	//	r.coeffRef(i, 2) += a.coeffRef(i, 0) * b.coeffRef(0, 2);

// 	//	r.coeffRef(i, 0) += a.coeffRef(i, 1) * b.coeffRef(1, 0);
// 	//	r.coeffRef(i, 1) += a.coeffRef(i, 1) * b.coeffRef(1, 1);
// 	//	r.coeffRef(i, 2) += a.coeffRef(i, 1) * b.coeffRef(1, 2);

// 	//	r.coeffRef(i, 0) += a.coeffRef(i, 2) * b.coeffRef(2, 0);
// 	//	r.coeffRef(i, 1) += a.coeffRef(i, 2) * b.coeffRef(2, 1);
// 	//	r.coeffRef(i, 2) += a.coeffRef(i, 2) * b.coeffRef(2, 2);
// 	//}
// 	//SSE3x3v1(a, b, r);
// 	//SSE3x3v2(a, b, r);
// 	//AVX3x3v1(a, b, r);
// 	//AVX3x3v2(a, b, r);
// 	//AVX3x3v3(a, b, r);
// 	AVX3x3v4(a, b, r);
// 	//AVX3x3v5(a, b, r);

// }

// void Tools::Multiply4x4(tMatrix& a, tMatrix& b, tMatrix& r) {
// 	AVX4x4v1(a, b, r);
// }

// void Tools::SSE3x3v1(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r) {
// 	double _bg1[2] = { 0, 0 };
// 	double _bg2[2] = { 0, 0 };
// 	double _bh[2] = { 0, 0 };

// 	{
// 		// AE
// 		auto v1 = _mm_set_pd(a.coeffRef(0, 0), a.coeffRef(0, 1));
// 		auto v2 = _mm_set_pd(a.coeffRef(1, 0), a.coeffRef(1, 1));

// 		auto v3 = _mm_loadr_pd(b.data());
// 		auto v4 = _mm_loadr_pd(b.data() + 3);

// 		auto v13 = _mm_mul_pd(v1, v3);
// 		auto v14 = _mm_mul_pd(v1, v4);

// 		auto v131 = _mm_unpackhi_pd(v13, v14);
// 		auto v141 = _mm_unpacklo_pd(v13, v14);

// 		auto r1 = _mm_add_pd(v131, v141);
// 		_mm_storel_pd(r.data(), r1);
// 		_mm_storeh_pd(r.data() + 3, r1);

// 		auto v23 = _mm_mul_pd(v2, v3);
// 		auto v24 = _mm_mul_pd(v2, v4);

// 		auto v231 = _mm_unpackhi_pd(v23, v24);
// 		auto v241 = _mm_unpacklo_pd(v23, v24);

// 		auto r2 = _mm_add_pd(v231, v241);
// 		_mm_storel_pd(r.data() + 1, r2);
// 		_mm_storeh_pd(r.data() + 4, r2);
// 	}

// 	{
// 		auto B  = _mm_loadr_pd(a.data() + 6);
// 		auto RB = _mm_load_pd(a.data() + 6); // reverse b
// 		auto G  = _mm_set_pd(b.coeffRef(2, 0), b.coeffRef(2, 1));
// 		auto bg1 = _mm_mul_pd(B, G);
// 		_mm_storer_pd(_bg1, bg1);

// 		auto bg2 = _mm_mul_pd(RB, G);
// 		_mm_storer_pd(_bg2, bg2);

// 		r.coeffRef(0, 0) += _bg1[0];
// 		r.coeffRef(1, 1) += _bg1[1];
// 		r.coeffRef(1, 0) += _bg2[0];
// 		r.coeffRef(0, 1) += _bg2[1];
// 	}

// 	{
// 		// AF
// 		auto v1 = _mm_set_pd(a.coeffRef(0, 0), a.coeffRef(0, 1));
// 		auto v2 = _mm_set_pd(a.coeffRef(1, 0), a.coeffRef(1, 1));

// 		auto f = _mm_loadr_pd(b.data() + 6);
// 		auto v1f = _mm_mul_pd(v1, f);
// 		auto v2f = _mm_mul_pd(v2, f);

// 		auto v1f1 = _mm_unpackhi_pd(v2f, v1f);
// 		auto v2f1 = _mm_unpacklo_pd(v2f, v1f);

// 		auto af = _mm_add_pd(v1f1, v2f1);
// 		_mm_storer_pd(r.data() + 6, af);
// 	}
// 	{
// 		//BH
// 		auto B = _mm_loadr_pd(a.data() + 6);
// 		auto H = _mm_set1_pd(a.coeffRef(2, 2));

// 		auto bh = _mm_mul_pd(B, H);
// 		_mm_storer_pd(_bh, bh);
// 		r.coeffRef(0, 2) += a.coeffRef(0, 2) * b.coeffRef(2,2);
// 		r.coeffRef(1, 2) += a.coeffRef(1, 2) * b.coeffRef(2, 2);
// 	}

// 	{
// 		//CE
// 		auto C = _mm_set_pd(a.coeffRef(2, 0), a.coeffRef(2, 1));
// 		auto E1 = _mm_loadr_pd(b.data());
// 		auto E2 = _mm_loadr_pd(b.data() + 3);

// 		auto ce1 = _mm_mul_pd(C, E1);
// 		auto ce2 = _mm_mul_pd(C, E2);

// 		auto ce11 = _mm_unpackhi_pd(ce2, ce1);
// 		auto ce12 = _mm_unpacklo_pd(ce2, ce1);

// 		auto ce = _mm_add_pd(ce11, ce12);
// 		_mm_storeh_pd(r.data() + 2, ce);
// 		_mm_storel_pd(r.data() + 5, ce);
// 	}

// 	{
// 		// DG
// 		auto G = _mm_set_pd(b.coeffRef(2, 0), b.coeffRef(2, 1));
// 		auto D = _mm_set_pd(a.coeffRef(2, 2), a.coeffRef(2, 2));
// 		auto dg = _mm_mul_pd(D, G);
// 		double _dg[2] = { 0, 0 };
// 		_mm_storer_pd(_dg, dg);

// 		//r.coeffRef(2, 0) = r.coeffRef(2, 0) + _dg[0];
// 		//r.coeffRef(2, 1) = r.coeffRef(2, 1) + _dg[1];

// 		r.data()[2] = r.data()[2] + _dg[0];
// 		r.data()[5] = r.data()[5] + _dg[1];

// 	}
// 	{
// 		// CF + DH
// 		auto C = _mm_set_pd(a.coeffRef(2, 0), a.coeffRef(2, 1));
// 		auto f = _mm_loadr_pd(b.data() + 6);
// 		auto cf = _mm_mul_pd(C, f);
// 		C = _mm_unpacklo_pd(cf, cf);
// 		auto cf1 = _mm_add_pd(C, cf);
// 		_mm_storeh_pd(r.data() + 8, cf1);
// 		r.coeffRef(2, 2) += a.coeffRef(2, 2) * b.coeffRef(2, 2);
// 	}
// }

// void Tools::SSE3x3v2(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r) {
// 	double _bg1[2] = { 0, 0 };
// 	double _bg2[2] = { 0, 0 };
// 	double _bh[2] = { 0, 0 };

// 	{
// 		// AE
// 		auto v1 = _mm_set_pd(a.coeffRef(0, 0), a.coeffRef(0, 1));
// 		auto v2 = _mm_set_pd(a.coeffRef(1, 0), a.coeffRef(1, 1));

// 		auto v3 = _mm_loadr_pd(b.data());
// 		auto v4 = _mm_loadr_pd(b.data() + 3);

// 		auto v13 = _mm_mul_pd(v1, v3);
// 		auto v14 = _mm_mul_pd(v1, v4);

// 		auto v131 = _mm_unpackhi_pd(v13, v14);
// 		auto v141 = _mm_unpacklo_pd(v13, v14);

// 		auto r1 = _mm_add_pd(v131, v141);
// 		_mm_storel_pd(r.data(), r1);
// 		_mm_storeh_pd(r.data() + 3, r1);

// 		auto v23 = _mm_mul_pd(v2, v3);
// 		auto v24 = _mm_mul_pd(v2, v4);

// 		auto v231 = _mm_unpackhi_pd(v23, v24);
// 		auto v241 = _mm_unpacklo_pd(v23, v24);

// 		auto r2 = _mm_add_pd(v231, v241);
// 		_mm_storel_pd(r.data() + 1, r2);
// 		_mm_storeh_pd(r.data() + 4, r2);

// 		// AF
// 		auto f = _mm_loadr_pd(b.data() + 6);
// 		auto v1f = _mm_mul_pd(v1, f);
// 		auto v2f = _mm_mul_pd(v2, f);

// 		auto v1f1 = _mm_unpackhi_pd(v2f, v1f);
// 		auto v2f1 = _mm_unpacklo_pd(v2f, v1f);

// 		auto af = _mm_add_pd(v1f1, v2f1);
// 		_mm_storer_pd(r.data() + 6, af);

// 	}

// 	{
// 		auto B = _mm_loadr_pd(a.data() + 6);
// 		auto RB = _mm_load_pd(a.data() + 6); // reverse b
// 		auto G = _mm_set_pd(b.coeffRef(2, 0), b.coeffRef(2, 1));
// 		auto bg1 = _mm_mul_pd(B, G);
// 		_mm_storer_pd(_bg1, bg1);

// 		auto bg2 = _mm_mul_pd(RB, G);
// 		_mm_storer_pd(_bg2, bg2);

// 		r.coeffRef(0, 0) += _bg1[0];
// 		r.coeffRef(1, 1) += _bg1[1];
// 		r.coeffRef(1, 0) += _bg2[0];
// 		r.coeffRef(0, 1) += _bg2[1];
// 	}

// 	{
// 		//BH
// 		auto B = _mm_loadr_pd(a.data() + 6);
// 		auto H = _mm_set1_pd(a.coeffRef(2,2));

// 		auto bh = _mm_mul_pd(B, H);
// 		_mm_storer_pd(_bh, bh);
// 		r.coeffRef(0, 2) += _bh[0];
// 		r.coeffRef(1, 2) += _bh[1];
// 	}

// 	{
// 		//CE
// 		auto C = _mm_set_pd(a.coeffRef(2, 0), a.coeffRef(2, 1));
// 		auto E1 = _mm_loadr_pd(b.data());
// 		auto E2 = _mm_loadr_pd(b.data() + 3);

// 		auto ce1 = _mm_mul_pd(C, E1);
// 		auto ce2 = _mm_mul_pd(C, E2);

// 		auto ce11 = _mm_unpackhi_pd(ce2, ce1);
// 		auto ce12 = _mm_unpacklo_pd(ce2, ce1);

// 		auto ce = _mm_add_pd(ce11, ce12);
// 		_mm_storeh_pd(r.data() + 2, ce);
// 		_mm_storel_pd(r.data() + 5, ce);
// 	}

// 	{
// 		// DG
// 		auto G = _mm_set_pd(b.coeffRef(2, 0), b.coeffRef(2, 1));
// 		auto D = _mm_set_pd(a.coeffRef(2, 2), a.coeffRef(2, 2));
// 		auto dg = _mm_mul_pd(D, G);
// 		double _dg[2] = { 0, 0 };
// 		_mm_storer_pd(_dg, dg);

// 		r.data()[2] = r.data()[2] + _dg[0];
// 		r.data()[5] = r.data()[5] + _dg[1];

// 	}
// 	{
// 		// CF + DH
// 		auto C = _mm_set_pd(a.coeffRef(2, 0), a.coeffRef(2, 1));
// 		auto f = _mm_loadr_pd(b.data() + 6);
// 		auto cf = _mm_mul_pd(C, f);
// 		C = _mm_unpacklo_pd(cf, cf);
// 		auto cf1 = _mm_add_pd(C, cf);
// 		_mm_storeh_pd(r.data() + 8, cf1);
// 		r.coeffRef(2, 2) += a.coeffRef(2, 2) * b.coeffRef(2, 2);
// 	}
// }

// void Tools::AVX3x3v1(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r) {

// 	auto zero = _mm256_setzero_pd();

// 	auto v1 = _mm256_set_pd(0, a.coeffRef(0, 2), a.coeffRef(0, 1), a.coeffRef(0, 0));

// 	auto v5 = _mm256_load_pd(b.data());
// 	auto v6 = _mm256_load_pd(b.data() + 3);
// 	auto v7 = _mm256_load_pd(b.data() + 6);

// 	auto v15 = _mm256_mul_pd(v1, v5);
// 	auto v16 = _mm256_mul_pd(v1, v6);

// 	auto h1 = _mm256_hadd_pd(v16, v15);
// 	auto h11 = _mm256_extractf128_pd(h1, 1);
// 	auto h12 = _mm256_extractf128_pd(h1, 0);
// 	auto h11_add_12 = _mm_add_pd(h11, h12);
// 	double r1[2] = { 0, 0 };
// 	_mm_store_pd(r1, h11_add_12);
// 	r.data()[0] = r1[1];
// 	r.data()[3] = r1[0];

// 	auto v17 = _mm256_mul_pd(v1, v7);
// 	h1 = _mm256_hadd_pd(zero, v17);
// 	h11 = _mm256_extractf128_pd(h1, 1);
// 	h12 = _mm256_extractf128_pd(h1, 0);
// 	h11_add_12 = _mm_add_pd(h11, h12);
// 	_mm_store_pd(r1, h11_add_12);
// 	r.data()[6] = r1[1];

// 	auto v2 = _mm256_set_pd(0, a.coeffRef(1, 2), a.coeffRef(1, 1), a.coeffRef(1, 0));

// 	auto v25 = _mm256_mul_pd(v2, v5);
// 	auto v26 = _mm256_mul_pd(v2, v6);

// 	auto h2 = _mm256_hadd_pd(v26, v25);
// 	auto h21 = _mm256_extractf128_pd(h2, 1);
// 	auto h22 = _mm256_extractf128_pd(h2, 0);
// 	auto h21_add_22 = _mm_add_pd(h21, h22);
// 	_mm_store_pd(r1, h21_add_22);
// 	r.coeffRef(1, 0) = r1[1];
// 	r.coeffRef(1, 1) = r1[0];
// 	;
// 	auto v27 = _mm256_mul_pd(v2, v7);
// 	h2 = _mm256_hadd_pd(zero, v27);
// 	h21 = _mm256_extractf128_pd(h2, 1);
// 	h22 = _mm256_extractf128_pd(h2, 0);
// 	h21_add_22 = _mm_add_pd(h21, h22);
// 	_mm_store_pd(r1, h21_add_22);
// 	r.data()[7]= r1[1];

// 	auto v3 = _mm256_set_pd(0, a.coeffRef(2, 2), a.coeffRef(2, 1), a.coeffRef(2, 0));

// 	auto v35 = _mm256_mul_pd(v3, v5);
// 	auto v36 = _mm256_mul_pd(v3, v6);
// 	auto v37 = _mm256_mul_pd(v3, v7);

// 	auto h3 = _mm256_hadd_pd(v36, v35);
// 	auto h31 = _mm256_extractf128_pd(h3, 1);
// 	auto h32 = _mm256_extractf128_pd(h3, 0);
// 	auto h31_add_32 = _mm_add_pd(h31, h32);
// 	_mm_store_pd(r1, h31_add_32);
// 	r.coeffRef(2, 0) = r1[1];
// 	r.coeffRef(2, 1) = r1[0];
// 	;

// 	h3 = _mm256_hadd_pd(zero, v37);
// 	h31 = _mm256_extractf128_pd(h3, 1);
// 	h32 = _mm256_extractf128_pd(h3, 0);
// 	h31_add_32 = _mm_add_pd(h31, h32);
// 	_mm_store_pd(r1, h31_add_32);
// 	r.data()[8] = r1[1];

// }

// __m128d hsum_double_avx(__m256d v) {
// 	__m128d vlow	= _mm256_castpd256_pd128(v);
// 	__m128d vhigh	= _mm256_extractf128_pd(v, 1); // high 128
// 		    vlow	= _mm_add_pd(vlow, vhigh);     // reduce down to 128

// 	__m128d high64 = _mm_unpackhi_pd(vlow, vlow);
// 	return  _mm_add_sd(vlow, high64);
// }

// void Tools::AVX3x3v2(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r) {

// 	double r1[2] = { 0, 0 };
// 	auto v5 = _mm256_load_pd(b.data());
// 	auto v6 = _mm256_load_pd(b.data() + 3);
// 	auto v7 = _mm256_load_pd(b.data() + 6);
// 	//__m256d vb[3] = { _mm256_load_pd(b.data()), _mm256_load_pd(b.data() + 3), _mm256_load_pd(b.data() + 6) };
// 	for(int i = 0; i < 3; ++i) {
// 		auto v1 = _mm256_set_pd(0, a.coeffRef(i, 2), a.coeffRef(i, 1), a.coeffRef(i, 0));

// 		__m256d v15 = _mm256_mul_pd(v1, v5);
// 		__m256d v16 = _mm256_mul_pd(v1, v6);

// 		__m256d h1			= _mm256_hadd_pd(v16, v15);
// 		__m128d h1_high		= _mm256_extractf128_pd(h1, 1);
// 		__m128d h1_low		= _mm256_castpd256_pd128(h1);
// 		__m128d h11_add_12	= _mm_add_pd(h1_high, h1_low);

// 		_mm_store_pd(r1, h11_add_12);
// 		r.coeffRef(i, 0) = r1[1];
// 		r.coeffRef(i, 1) = r1[0];

// 		__m256d v17	= _mm256_mul_pd(v1, v7);
// 		h1			= _mm256_hadd_pd(v17, v17);
// 		h1_high		= _mm256_extractf128_pd(h1, 1);
// 		h1_low		= _mm256_castpd256_pd128(h1);
// 		h11_add_12	= _mm_add_pd(h1_high, h1_low);
// 		_mm_storeh_pd(r.data() + 6 + i, h11_add_12);

// 		//auto v15 = _mm256_mul_pd(v1, v5);
// 		//auto sum1 = hsum_double_avx(v15);
// 		//_mm_storel_pd(r1, sum1);
// 		//r.coeffRef(i, 0) = r1[0];

// 		//auto v16 = _mm256_mul_pd(v1, v6);
// 		//sum1 = hsum_double_avx(v16);
// 		//_mm_storel_pd(r1, sum1);
// 		//r.coeffRef(i, 1) = r1[0];

// 		//auto v17 = _mm256_mul_pd(v1, v7);
// 		//sum1 = hsum_double_avx(v17);
// 		//_mm_storel_pd(r1, sum1);
// 		//r.coeffRef(i, 2) = r1[0];

// 	}
// }

// void Tools::AVX3x3v3(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r) {

// 	__m256d _bv[3] = {
// 		_mm256_set_pd(0, b.coeffRef(0, 2), b.coeffRef(0, 1), b.coeffRef(0, 0)),
// 		_mm256_set_pd(0, b.coeffRef(1, 2), b.coeffRef(1, 1), b.coeffRef(1, 0)),
// 		_mm256_set_pd(0, b.coeffRef(2, 2), b.coeffRef(2, 1), b.coeffRef(2, 0))
// 	};

// 	double _d[4] = { 0, 0, 0, 0 };
// 	__m256d _sum[3];
// 	__m256d r1;
// 	for (int i = 0; i < 3; ++i) {

// 		//for (int k = 0; k < 3; ++k) {
// 		//	auto r1 = _mm256_set1_pd(a.coeffRef(i, k));
// 		//	_sum[k] = _mm256_mul_pd(r1, _bv[k]);
// 		//}
// 		//for (int j = 1; j < 3; ++j) {
// 		//	_sum[0] = _mm256_add_pd(_sum[0], _sum[j]);
// 		//}

// 		r1 = _mm256_set1_pd(a.coeffRef(i, 0));
// 		_sum[0] = _mm256_mul_pd(r1, _bv[0]);
// 		r1 = _mm256_set1_pd(a.coeffRef(i, 1));
// 		_sum[1] = _mm256_mul_pd(r1, _bv[1]);
// 		r1 = _mm256_set1_pd(a.coeffRef(i, 2));
// 		_sum[2] = _mm256_mul_pd(r1, _bv[2]);

// 		_sum[0] = _mm256_add_pd(_sum[0], _sum[1]);
// 		_sum[0] = _mm256_add_pd(_sum[0], _sum[2]);

// 		_mm256_store_pd(_d, _sum[0]);

// 		r.coeffRef(i, 0) = _d[0];
// 		r.coeffRef(i, 1) = _d[1];
// 		r.coeffRef(i, 2) = _d[2];

// 	}
// }

// void Tools::AVX3x3v4(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r) {

// 	__m256d _av[3] = {
// 		_mm256_load_pd(a.data()),
// 		_mm256_load_pd(a.data() + 3),
// 		_mm256_load_pd(a.data() + 6),
// 	};

// 	double  _d[4] = { 0, 0, 0, 0 };
// 	__m256d _sum[3];
// 	__m256d r1;
// 	for (int i = 0; i < 3; ++i) {
// 		r1 = _mm256_set1_pd(b.coeffRef(0, i));
// 		_sum[0] = _mm256_mul_pd(r1, _av[0]);
// 		r1 = _mm256_set1_pd(b.coeffRef(1, i));
// 		_sum[1] = _mm256_mul_pd(r1, _av[1]);
// 		r1 = _mm256_set1_pd(b.coeffRef(2, i));
// 		_sum[2] = _mm256_mul_pd(r1, _av[2]);

// 		_sum[0] = _mm256_add_pd(_sum[0], _sum[1]);
// 		_sum[0] = _mm256_add_pd(_sum[0], _sum[2]);

// 		_mm256_store_pd(_d, _sum[0]);

// 		r.data()[i * 3 + 0] = _d[0];
// 		r.data()[i * 3 + 1] = _d[1];
// 		r.data()[i * 3 + 2] = _d[2];

// 		//r.coeffRef(1, i) = _d[1];
// 		//r.coeffRef(2, i) = _d[2];
// 	}

// }

// void Tools::AVX3x3v5(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r) {

// 	double r1[2] = { 0, 0 };
// 	__m256d _av[3] = {
// 		_mm256_set_pd(0, a.coeffRef(0, 2), a.coeffRef(0, 1), a.coeffRef(0, 0)),
// 		_mm256_set_pd(0, a.coeffRef(1, 2), a.coeffRef(1, 1), a.coeffRef(1, 0)),
// 		_mm256_set_pd(0, a.coeffRef(2, 2), a.coeffRef(2, 1), a.coeffRef(2, 0))
// 	};

// 	__m256d h1;
// 	__m128d h1_high;
// 	__m128d h1_low;
// 	__m128d h11_add_12;

// 	// this runs slow, might because:
// 	// there are too many separate variables between two iterations
// 	// so it is not cache friendly
// 	for (int i = 0; i < 3; ++i) {
// 		auto v5 = _mm256_load_pd(b.data());
// 		__m256d v15 = _mm256_mul_pd(_av[i], v5);

// 		auto v6 = _mm256_load_pd(b.data() + 3);
// 		__m256d v16 = _mm256_mul_pd(_av[i], v6);

// 		h1			= _mm256_hadd_pd(v16, v15);
// 		h1_high		= _mm256_extractf128_pd(h1, 1);
// 		h1_low		= _mm256_castpd256_pd128(h1);
// 		h11_add_12	= _mm_add_pd(h1_high, h1_low);

// 		_mm_store_pd(r1, h11_add_12);
// 		r.coeffRef(i, 0) = r1[1];
// 		r.coeffRef(i, 1) = r1[0];

// 		v6 = _mm256_load_pd(b.data() + 6);
// 		__m256d v17 = _mm256_mul_pd(_av[i], v6);

// 		h1			= _mm256_hadd_pd(v17, v17);
// 		h1_high		= _mm256_extractf128_pd(h1, 1);
// 		h1_low		= _mm256_castpd256_pd128(h1);
// 		h11_add_12	= _mm_add_pd(h1_high, h1_low);
// 		_mm_storeh_pd(r.data() + 6 + i, h11_add_12);

// 	}

// }

// void Tools::AVX4x4v1(const tMatrix& a, const tMatrix& b, tMatrix& r) {

// 	__m256d _av[4] = {
// 		_mm256_load_pd(a.data()),
// 		_mm256_load_pd(a.data() + 4),
// 		_mm256_load_pd(a.data() + 8),
// 		_mm256_load_pd(a.data() + 12),
// 	};

// 	double  _d[4] = { 0, 0, 0, 0 };
// 	__m256d _sum[4];
// 	__m256d r1;
// 	for (int i = 0; i < 4; ++i) {
// 		r1 = _mm256_set1_pd(b.coeffRef(0, i));
// 		_sum[0] = _mm256_mul_pd(r1, _av[0]);
// 		r1 = _mm256_set1_pd(b.coeffRef(1, i));
// 		_sum[1] = _mm256_mul_pd(r1, _av[1]);
// 		r1 = _mm256_set1_pd(b.coeffRef(2, i));
// 		_sum[2] = _mm256_mul_pd(r1, _av[2]);
// 		r1 = _mm256_set1_pd(b.coeffRef(3, i));
// 		_sum[3] = _mm256_mul_pd(r1, _av[3]);

// 		_sum[0] = _mm256_add_pd(_sum[0], _sum[1]);
// 		_sum[0] = _mm256_add_pd(_sum[0], _sum[2]);
// 		_sum[0] = _mm256_add_pd(_sum[0], _sum[3]);

// 		_mm256_store_pd(_d, _sum[0]);

// 		r.data()[i * 4 + 0] = _d[0];
// 		r.data()[i * 4 + 1] = _d[1];
// 		r.data()[i * 4 + 2] = _d[2];
// 		r.data()[i * 4 + 3] = _d[3];
// 	}
// }

// void Tools:: AVX4x4v1_3mat(const tMatrix& a, const tMatrix& b, const tMatrix& c, tMatrix& r) {
// 	AVX4x4v1(a, b, t);
// 	AVX4x4v1(t, c, r);
// }

void Tools::AVX4x4v1_6mat(const tMatrix& a, const tMatrix& b, const tMatrix& c, const tMatrix& d, const tMatrix& e, const tMatrix& f,
						  tMatrix& r)
{
	AVX4x4v1_3mat(a, b, c, t1);
	AVX4x4v1_3mat(d, e, f, t2);
	AVX4x4v1(t1, t2, r);
}

void Tools::MatMul4x1(const tMatrix& m, const tVector& v, tVector& r)
{
	r.noalias() = m * v;
}
// void Tools::MatMul4x1(const tMatrix& m, const tVector& v, tVector& r)
// {
// 	__m256d _av[4] = {
// 		_mm256_load_pd(m.data()),
// 		_mm256_load_pd(m.data() + 4),
// 		_mm256_load_pd(m.data() + 8),
// 		_mm256_load_pd(m.data() + 12),
// 	};
// 	double  _d[4] = { 0, 0, 0, 0 };
// 	__m256d _sum[4];
// 	__m256d r1;

// 	r1 = _mm256_set1_pd(v.coeffRef(0));
// 	_sum[0] = _mm256_mul_pd(r1, _av[0]);
// 	r1 = _mm256_set1_pd(v.coeffRef(1));
// 	_sum[1] = _mm256_mul_pd(r1, _av[1]);
// 	r1 = _mm256_set1_pd(v.coeffRef(2));
// 	_sum[2] = _mm256_mul_pd(r1, _av[2]);
// 	r1 = _mm256_set1_pd(v.coeffRef(3));
// 	_sum[3] = _mm256_mul_pd(r1, _av[3]);

// 	_sum[0] = _mm256_add_pd(_sum[0], _sum[1]);
// 	_sum[0] = _mm256_add_pd(_sum[0], _sum[2]);
// 	_sum[0] = _mm256_add_pd(_sum[0], _sum[3]);

// 	_mm256_store_pd(_d, _sum[0]);

// 	r.data()[0] = _d[0];
// 	r.data()[1] = _d[1];
// 	r.data()[2] = _d[2];
// 	r.data()[3] = _d[3];
// }
