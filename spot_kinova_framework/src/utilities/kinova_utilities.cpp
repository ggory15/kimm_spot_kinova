#include <cstdlib>
#include "spot_kinova_framework/utilities/kinova_utilities.hpp"
#include "stdio.h"

using namespace std;

//Helper function to parse program arguments
ExampleArgs ParseExampleArguments(int argc, char *argv[])
{
    cxxopts::Options options(argv[0], "Kortex example");
    
    options.add_options()
        ("ip", "IP address of destination", cxxopts::value<std::string>()->default_value("192.168.1.10"))
        ("h,help", "Print help")
        ("u,username", "username to login", cxxopts::value<std::string>()->default_value("admin"))
        ("p,password", "password to login", cxxopts::value<std::string>()->default_value("admin"))
    ;

    ExampleArgs resultArgs;

    try
    {
        auto parsed_options = options.parse(argc, argv);

        if (parsed_options.count("help"))
        {
            std::cout << options.help() << std::endl;
            exit(EXIT_SUCCESS);
        }

        resultArgs.ip_address = parsed_options["ip"].as<std::string>();
        resultArgs.username = parsed_options["username"].as<std::string>();
        resultArgs.password = parsed_options["password"].as<std::string>();
    }
    catch(const cxxopts::OptionException& exception)
    {
        std::cerr << exception.what() << std::endl;
        exit(EXIT_FAILURE);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        exit(EXIT_FAILURE);
    }
    
    return resultArgs;
}
double wrapRadiansFromMinusPiToPi(double rad_not_wrapped, int& number_of_turns)
{
    bool properly_wrapped = false;
    number_of_turns = 0;
    do 
    {
        if (rad_not_wrapped > M_PI)
        {
            number_of_turns += 1;
            rad_not_wrapped -= 2.0*M_PI;
        }
        else if (rad_not_wrapped < -M_PI)
        {
            number_of_turns -= 1;
            rad_not_wrapped += 2.0*M_PI;
        }
        else
        {
            properly_wrapped = true;
        }
    } while(!properly_wrapped);
    return rad_not_wrapped;
}

double wrapRadiansFromMinusPiToPi(double rad_not_wrapped)
{
    int n;
    return wrapRadiansFromMinusPiToPi(rad_not_wrapped, n);
}