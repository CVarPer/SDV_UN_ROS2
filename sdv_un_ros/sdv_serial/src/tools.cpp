#include <tools.h>

double tools::constrain(double value, double minval, double maxval)
{
    double r = value;
    if (value > maxval)
        r = maxval;
    if (value < minval)
        r = minval;
    return r;
}


vector<string> tools::getArgs(string data)
{
    stringstream ss(data);
    string d;
    vector<string> vdata;

    while (ss >> d)
        vdata.push_back(d);

    return vdata;
}

string tools::cleanString(string s)
{
    s.erase(std::remove(s.begin(), s.end(), '\r'), s.end());
    s.erase(std::remove(s.begin(), s.end(), '\n'), s.end());

    return s;
}
