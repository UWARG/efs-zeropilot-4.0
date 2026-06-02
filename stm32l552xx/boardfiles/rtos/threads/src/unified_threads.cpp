#include "unified_threads.hpp"

void initThreads()
{
  startUpInitThreads();
  amInitThreads();
  smInitThreads();
  tmInitThreads();
}
