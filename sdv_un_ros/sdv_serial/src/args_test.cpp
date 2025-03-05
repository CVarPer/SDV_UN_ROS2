#include <tools.h>

using namespace std;

int main(int argc, char **argv)
{
    string s = "<13 [0.0000, 1, 1], [0.0000, 1, 1], [0.0000, 1, 1], [0.0000, 1, 1]\n\r";
    string s1 = tools::cleanString(s);
    vector<string> arguments = tools::getArgs(s1);

    cout << s1 << "\n\r";
    for(int i = 0; i < arguments.size(); i++)
    {
        cout << "  " << arguments[i] << "\n\r";
    }
}