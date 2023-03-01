#include <iostream>
#include "protoc/msg.pb.h"


int main(int argc, char const *argv[])
{   
    IM::EM::Content msg;

    msg.set_id(10);
    msg.set_str("sshuaige"),
    std::cout << msg.id() << '\n' << msg.str() << std::endl;

    return 0;
}
