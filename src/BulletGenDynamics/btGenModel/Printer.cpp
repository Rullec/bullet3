#include "Printer.h"

bool Printer::AskForUserConfirmation(const std::string &question)
{
    char ans = 'N';
    do
    {
        std::cout << question << std::endl;
        std::cout << "Do you want to continue (Y/N)?\n";
        std::cout << "You must type a 'Y' or an 'N' :";
        std::cin >> ans;
    } while ((ans != 'Y') && (ans != 'N'));
    if (ans == 'Y')
        return true;
    else
        return false;
}