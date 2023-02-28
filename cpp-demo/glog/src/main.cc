#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_string(name,"gcx","your name");
int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    gflags::ParseCommandLineFlags(&argc,&argv,true);
    int i = 0;
    while (i < 10)
    {
        LOG(ERROR) << "hello world" << i << FLAGS_name;
        i++;
    }
    return 0;
}
