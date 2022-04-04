/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <QtWidgets/QApplication>
#include "mainwindow.h"
#include "winmain.h"

int main(int argc, char *argv[])
{
    std::locale::global( std::locale( "" ) );
    
    char** argend = argc + argv;

    QCoreApplication::setOrganizationName("Parsian");
    QCoreApplication::setOrganizationDomain("parsian-robotics.com");
    QCoreApplication::setApplicationName("grSim");
    QApplication a(argc, argv);
    MainWindow w;

    if (std::find(argv, argend, std::string("--headless")) != argend
        || std::find(argv, argend, std::string("-H")) != argend) {
        // enable headless mode
        w.hide();
        w.setIsGlEnabled(false);
    } else {
        // Run normally
        w.show();
    }
    if(std::find(argv, argend, std::string("--atkfault")) != argend)
        w.withGoalKick(true);
    if(std::find(argv, argend, std::string("--xlr8")) != argend)
        w.fullSpeed(true);
    if (std::find(argv, argend, std::string("--fast")) != argend)
        w.setFast();

    return QApplication::exec();
}
