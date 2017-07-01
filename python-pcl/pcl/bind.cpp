#include "bind.h"

boost::signals2::signal<void (void*)> g_signal;

void some_callback(void* some_ptr) {
  std::cout << "Hello from some_callback !" << std::endl;
  std::cout << "some_ptr: " << some_ptr << std::endl;
}

boost::signals2::connection register_callback(boost::function<void (void*)> callback) {
  return g_signal.connect(callback);
}

int main(int argc, const char* argv[]) {

  std::cout << "(void*)argv: " << (void*)argv << std::endl;

  auto callback = boost::bind(some_callback, _1);
  auto conn = register_callback(callback);

  g_signal((void*)argv);

  std::cout << "End of execution..." << std::endl;

  return 0;
}
