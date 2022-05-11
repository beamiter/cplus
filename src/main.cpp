#include <Eigen/Dense>
#include <iostream>
#include <type_traits>
#include <vector>

#include "robot_dynamics/car_model.h"
#include "robot_dynamics/discrete_dynamics.h"
#include "robot_dynamics/dynamics.h"
#include "robot_dynamics/functionbase.h"

using namespace std;

struct A {
  virtual void test() const { cout << ">>>>>>>>> A\n"; }
  virtual void haha() const { cout << ">>>>>>>>> A\n"; }
};
struct AA : A {
  virtual void test() const { cout << ">>>>>>>>> B\n"; }
};
struct AAA : AA {};

template <typename T> class B {
  static_assert(std::is_base_of<A, T>::value, "not derived from A");
  typedef typename std::enable_if<std::is_base_of<A, T>::value, T>::type base;

public:
  // virtual void test() = 0;
  void fund() { cout << ">>>>>>> basic function\n"; }
  void haha() { cout << "daidai\n"; }
};

template <typename T> class C : public B<T> {
  void test() const { cout << ">>>>>>> c\n"; }
};

using Eigen::MatrixXd;

int main() {
  AAA aaa;
  aaa.test();
  aaa.haha();
  const A &a = aaa;
  a.test();
  a.haha();
  cout << std::is_base_of<A, AA>::value;

  char str[50];

  strcpy(str, "This is string.h library function");
  puts(str);

  memset(&str, '$', 7 * sizeof(char));
  puts(str);

  cout << sizeof(char) << ", " << sizeof(float) << endl;

  float Location[3];
  cout << sizeof(Location) << endl;
  memset(&Location, 0, 3 * sizeof(float));
  cout << sizeof(Location) << endl;

  std::string s = "00000";
  cout << s << ", " << s.size()<< endl;
  s.resize(8, '0');
  cout << s << s.size() <<  endl;

}
