#ifndef __CINTERVAL_HPP__
#define __CINTERVAL_HPP__

// The class cInterval implements the basic interval operations on real
// intervals

class cInterval {
public:
	double i[2];

	cInterval();
	cInterval(double v);
	cInterval(double ll, double rr);

	cInterval operator+(const cInterval& in);
	cInterval operator-(const cInterval& in);
	cInterval& operator+=(const cInterval& in);
	cInterval& operator-=(const cInterval& in);
	cInterval operator*(const cInterval& in);
	cInterval operator*(const double d);
	cInterval operator/(const cInterval& in);
	bool operator^(const cInterval& in);
	cInterval operator-();

	double getAbsLower();
	double getAbsUpper();
	bool contains(double v);
	void bound(double v);
	void bound(cInterval& j);

	void print();
	double center() { return 0.5 * (i[0] + i[1]); }
	double diameter() { return i[1] - i[0]; }
};

cInterval operator*(double d, cInterval in);
cInterval operator+(double d, cInterval in);

#endif  // __CINTERVAL_HPP__