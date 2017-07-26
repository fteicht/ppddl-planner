/* -*-C++-*- */
/*
 * Copyright 2003-2005 Carnegie Mellon University and Rutgers University
 * Copyright 2007 Håkan Younes
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "rational.h"
#include <stdexcept>
#include <cstdlib>
#include <cmath>
#include <cfloat>
#include <climits>


/* ====================================================================== */
/* Rational */

/* By default, rational numbers are printed as fractions */
bool Rational::asDouble_ = false;

/* Returns the greatest common devisor of the two integers. */
static int gcd(int n, int m) {
  int a = abs(n);
  int b = abs(m);
  while (b > 0) {
    int c = b;
    b = a % b;
    a = c;
  }
  return a;
}


/* Returns the least common multiplier of the two integers. */
static int lcm(int n, int m) {
  return n/gcd(n, m)*m;
}


/* Returns the multipliers for the two integers. */
std::pair<int, int> Rational::multipliers(int n, int m) {
  int f = lcm(n, m);
  return std::make_pair(f/n, f/m);
}


/* Constructs a rational number. */
Rational::Rational(int n, int m) {
  if (m == 0) {
    throw std::runtime_error("division by zero");
  } else {
    int d = gcd(n, m);
    numerator_ = n/d;
    denominator_ = m/d;
    if (denominator_ < 0) {
      numerator_ *= -1;
      denominator_ *= -1;
    }
  }
}


/* Constructs a rational number. */
Rational::Rational(const char* s)
  : numerator_(0) {
  const char* si = s;
  for (; *si != '\0' && *si != '.' && *si != '/'; si++) {
    numerator_ = 10*numerator_ + (*si - '0');
  }
  if (*si == '/') {
    denominator_ = 0;
    for (si++; *si != '\0'; si++) {
      denominator_ = 10*denominator_ + (*si - '0');
    }
    if (denominator_ == 0) {
      throw std::runtime_error("division by zero");
    }
    int d = gcd(numerator_, denominator_);
    numerator_ /= d;
    denominator_ /= d;
  } else if (*si == '.') {
    int a = numerator_;
    numerator_ = 0;
    denominator_ = 1;
    for (si++; *si != '\0'; si++) {
      numerator_ = 10*numerator_ + (*si - '0');
      denominator_ *= 10;
    }
    int d = gcd(numerator_, denominator_);
    numerator_ /= d;
    denominator_ /= d;
    numerator_ += a*denominator_;
  } else {
    denominator_ = 1;
  }
}


/* Constructs a rational number as a double-precision number */
Rational::Rational(double d) {
    int cnt = 0;
    int pow = 1;
    double val = d;
    int powmax = (int) (std::log(INT_MAX) / std::log(10));
    int valmax = (int) ((std::log(INT_MAX) - std::log((val > 0)?val:(-val))) / std::log(10));

    while ((std::fabs(val - ((int) val)) > DBL_EPSILON) && (cnt < (valmax - 1)) && (cnt < (powmax - 1)) && (cnt < (FLT_MAX_10_EXP - 1))) {
        val *= 10;
        pow *= 10;
        cnt++;
    }

    numerator_ = (int) val;
    denominator_ = pow;
}


/* Less-than comparison operator for rational numbers. */
bool operator<(const Rational& q, const Rational& p) {
  std::pair<int, int> m =
    Rational::multipliers(q.denominator(), p.denominator());
  return q.numerator()*m.first < p.numerator()*m.second;
}


/* Less-than-or-equal comparison operator for rational numbers. */
bool operator<=(const Rational& q, const Rational& p) {
  std::pair<int, int> m =
    Rational::multipliers(q.denominator(), p.denominator());
  return q.numerator()*m.first <= p.numerator()*m.second;
}


/* Equality comparison operator for rational numbers. */
bool operator==(const Rational& q, const Rational& p) {
  std::pair<int, int> m =
    Rational::multipliers(q.denominator(), p.denominator());
  return q.numerator()*m.first == p.numerator()*m.second;
}


/* Inequality comparison operator for rational numbers. */
bool operator!=(const Rational& q, const Rational& p) {
  std::pair<int, int> m =
    Rational::multipliers(q.denominator(), p.denominator());
  return q.numerator()*m.first != p.numerator()*m.second;
}


/* Greater-than-or-equal comparison operator for rational numbers. */
bool operator>=(const Rational& q, const Rational& p) {
  std::pair<int, int> m =
    Rational::multipliers(q.denominator(), p.denominator());
  return q.numerator()*m.first >= p.numerator()*m.second;
}


/* Greater-than comparison operator for rational numbers. */
bool operator>(const Rational& q, const Rational& p) {
  std::pair<int, int> m =
    Rational::multipliers(q.denominator(), p.denominator());
  return q.numerator()*m.first > p.numerator()*m.second;
}


/* Addition operator for rational numbers. */
Rational operator+(const Rational& q, const Rational& p) {
  std::pair<int, int> m =
    Rational::multipliers(q.denominator(), p.denominator());
  return Rational(q.numerator()*m.first + p.numerator()*m.second,
                  q.denominator()*m.first);
}


/* Subtraction operator for rational numbers. */
Rational operator-(const Rational& q, const Rational& p) {
  std::pair<int, int> m =
    Rational::multipliers(q.denominator(), p.denominator());
  return Rational(q.numerator()*m.first - p.numerator()*m.second,
                  q.denominator()*m.first);
}


/* Multiplication operator for rational numbers. */
Rational operator*(const Rational& q, const Rational& p) {
  int d1 = gcd(q.numerator(), p.denominator());
  int d2 = gcd(p.numerator(), q.denominator());
  return Rational((q.numerator()/d1)*(p.numerator()/d2),
                  (q.denominator()/d2)*(p.denominator()/d1));
}


/* Division operator for rational numbers. */
Rational operator/(const Rational& q, const Rational& p) {
  if (p == 0) {
    throw std::runtime_error("division by zero");
  }
  int d1 = gcd(q.numerator(), p.numerator());
  int d2 = gcd(p.denominator(), q.denominator());
  return Rational((q.numerator()/d1)*(p.denominator()/d2),
                  (q.denominator()/d2)*(p.numerator()/d1));
}


/* Output operator for rational numbers. */
std::ostream& operator<<(std::ostream& os, const Rational& q) {
  if (Rational::print_double()) {
    os << q.double_value();
  } else {
    os << q.numerator();
    if (q.denominator() != 1) {
      os << '/' << q.denominator();
    }
  }
  return os;
}
