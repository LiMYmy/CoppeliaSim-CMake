#include<iostream>
/* 一维三次样条插值
po vo ao to 初始点位置 速度 加速度 时间
p1 t1 中间点 位置 时间
p2 v2 a2 t2 最后点位置 速度 加速度 时间
t 输出时间
PP[3] 输出 位置 速度 加速度
*/
void TSpline_S_V_A_realtime(double p0, double v0, double a0, double t0,
	double p1, double t1, double p2, double v2,
	double a2, double t2, double t, double PP[3])
{
	double R[100];
	double p[10];
	double R_tmp;
	double b_R_tmp;
	double s;
	double smax;
	int a;
	int i;
	int jA;
	int k;
	signed char ipiv[10];
	R[0] = 1.0;
	R[10] = t0;
	smax = t0 * t0;
	R[20] = smax;
	smax *= t0;
	R[30] = smax;
	R[40] = smax * t0;
	R[50] = 0.0;
	R[60] = 0.0;
	R[70] = 0.0;
	R[80] = 0.0;
	R[90] = 0.0;
	R[1] = 0.0;
	R[11] = 1.0;
	R[21] = 2.0 * t0;
	R[31] = 3.0 * t0 * t0;
	R[41] = 4.0 * t0 * t0 * t0;
	R[51] = 0.0;
	R[61] = 0.0;
	R[71] = 0.0;
	R[81] = 0.0;
	R[91] = 0.0;
	R[2] = 0.0;
	R[12] = 0.0;
	R[22] = 2.0;
	R[32] = 6.0 * t0;
	R[42] = 12.0 * t0 * t0;
	R[52] = 0.0;
	R[62] = 0.0;
	R[72] = 0.0;
	R[82] = 0.0;
	R[92] = 0.0;
	R[3] = 1.0;
	R[13] = t1;
	smax = t1 * t1;
	R[23] = smax;
	s = smax * t1;
	R[33] = s;
	R_tmp = s * t1;
	R[43] = R_tmp;
	R[53] = 0.0;
	R[63] = 0.0;
	R[73] = 0.0;
	R[83] = 0.0;
	R[93] = 0.0;
	R[4] = 0.0;
	R[14] = 0.0;
	R[24] = 0.0;
	R[34] = 0.0;
	R[44] = 0.0;
	R[54] = 1.0;
	R[64] = t2;
	b_R_tmp = t2 * t2;
	R[74] = b_R_tmp;
	b_R_tmp *= t2;
	R[84] = b_R_tmp;
	R[94] = b_R_tmp * t2;
	R[5] = 0.0;
	R[15] = 0.0;
	R[25] = 0.0;
	R[35] = 0.0;
	R[45] = 0.0;
	R[55] = 0.0;
	R[65] = 1.0;
	R[75] = 2.0 * t2;
	R[85] = 3.0 * t2 * t2;
	R[95] = 4.0 * t2 * t2 * t2;
	R[6] = 0.0;
	R[16] = 0.0;
	R[26] = 0.0;
	R[36] = 0.0;
	R[46] = 0.0;
	R[56] = 0.0;
	R[66] = 0.0;
	R[76] = 2.0;
	R[86] = 6.0 * t2;
	R[96] = 12.0 * t2 * t2;
	R[7] = 0.0;
	R[17] = 0.0;
	R[27] = 0.0;
	R[37] = 0.0;
	R[47] = 0.0;
	R[57] = 1.0;
	R[67] = t1;
	R[77] = smax;
	R[87] = s;
	R[97] = R_tmp;
	R[8] = 0.0;
	R[18] = 1.0;
	R[28] = 2.0 * t1;
	R[38] = 3.0 * t1 * t1;
	R[48] = 4.0 * t1 * t1 * t1;
	R[58] = 0.0;
	R[68] = -1.0;
	R[78] = -2.0 * t1;
	R[88] = -3.0 * t1 * t1;
	R[98] = -4.0 * t1 * t1 * t1;
	R[9] = 0.0;
	R[19] = 0.0;
	R[29] = 2.0;
	R[39] = 6.0 * t1;
	R[49] = 12.0 * t1 * t1;
	R[59] = 0.0;
	R[69] = 0.0;
	R[79] = -2.0;
	R[89] = -6.0 * t1;
	R[99] = -12.0 * t1 * t1;
	p[0] = p0;
	p[1] = v0;
	p[2] = a0;
	p[3] = p1;
	p[4] = p2;
	p[5] = v2;
	p[6] = a2;
	p[7] = p1;
	p[8] = 0.0;
	p[9] = 0.0;
	for (i = 0; i < 10; i++) {
		ipiv[i] = static_cast<signed char>(i + 1);
	}
	for (int j{ 0 }; j < 9; j++) {
		int b_tmp;
		int c_R_tmp;
		int jp1j;
		int mmj_tmp;
		signed char i1;
		mmj_tmp = 8 - j;
		b_tmp = j * 11;
		jp1j = b_tmp + 2;
		jA = 10 - j;
		a = 0;
		smax = std::abs(R[b_tmp]);
		for (k = 2; k <= jA; k++) {
			s = std::abs(R[(b_tmp + k) - 1]);
			if (s > smax) {
				a = k - 1;
				smax = s;
			}
		}
		if (R[b_tmp + a] != 0.0) {
			if (a != 0) {
				jA = j + a;
				ipiv[j] = static_cast<signed char>(jA + 1);
				for (k = 0; k < 10; k++) {
					a = j + k * 10;
					smax = R[a];
					c_R_tmp = jA + k * 10;
					R[a] = R[c_R_tmp];
					R[c_R_tmp] = smax;
				}
			}
			i = (b_tmp - j) + 10;
			for (a = jp1j; a <= i; a++) {
				R[a - 1] /= R[b_tmp];
			}
		}
		jA = b_tmp;
		for (a = 0; a <= mmj_tmp; a++) {
			smax = R[(b_tmp + a * 10) + 10];
			if (smax != 0.0) {
				i = jA + 12;
				c_R_tmp = (jA - j) + 20;
				for (jp1j = i; jp1j <= c_R_tmp; jp1j++) {
					R[jp1j - 1] += R[((b_tmp + jp1j) - jA) - 11] * -smax;
				}
			}
			jA += 10;
		}
		i1 = ipiv[j];
		if (i1 != j + 1) {
			smax = p[j];
			p[j] = p[i1 - 1];
			p[i1 - 1] = smax;
		}
	}
	for (k = 0; k < 10; k++) {
		jA = 10 * k;
		if (p[k] != 0.0) {
			i = k + 2;
			for (a = i; a < 11; a++) {
				p[a - 1] -= p[k] * R[(a + jA) - 1];
			}
		}
	}
	for (k = 9; k >= 0; k--) {
		jA = 10 * k;
		smax = p[k];
		if (smax != 0.0) {
			smax /= R[k + jA];
			p[k] = smax;
			for (a = 0; a < k; a++) {
				p[a] -= p[k] * R[a + jA];
			}
		}
	}
	PP[0] = 0.0;
	PP[1] = 0.0;
	PP[2] = 0.0;
	if (t <= t1) {
		PP[0] = (((p[0] + p[1] * t) + p[2] * t * t) + p[3] * t * t * t) +
			p[4] * t * t * t * t;
		PP[1] =
			((p[1] + 2.0 * p[2] * t) + 3.0 * p[3] * t * t) + 4.0 * p[4] * t * t * t;
		PP[2] = (2.0 * p[2] + 6.0 * p[3] * t) + 12.0 * p[4] * t * t;
	}
	if (t > t1) {
		PP[0] = (((p[5] + p[6] * t) + p[7] * t * t) + p[8] * t * t * t) +
			p[9] * t * t * t * t;
		PP[1] =
			((p[6] + 2.0 * p[7] * t) + 3.0 * p[8] * t * t) + 4.0 * p[9] * t * t * t;
		PP[2] = (2.0 * p[7] + 6.0 * p[8] * t) + 12.0 * p[9] * t * t;
	}
}
