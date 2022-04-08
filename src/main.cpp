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
#include <QCommandLineParser>
#include <QCommandLineOption>
#include "mainwindow.h"
#include "winmain.h"
#include <string>

int main(int argc, char *argv[])
{
    std::locale::global( std::locale( "" ) );
    
    char** argend = argc + argv;

    QCoreApplication::setOrganizationName("Parsian");
    QCoreApplication::setOrganizationDomain("parsian-robotics.com");
    QCoreApplication::setApplicationName("grSim");
    QApplication a(argc, argv);
    MainWindow w;

    QCommandLineParser parser;
    QCommandLineOption idOption("id", "Set udp ports by id", "id");
    parser.addOption(idOption);
    QCommandLineOption setHeadless({"headless", "H"}, "Run without GUI");
    parser.addOption(setHeadless);
    QCommandLineOption setAtkfault("atkfault", "Sets withGoalKick to true");
    parser.addOption(setAtkfault);
    QCommandLineOption setFullSpeed("xlr8", "Sets fullSpeed to true");
    parser.addOption(setFullSpeed);
    QCommandLineOption setFast("fast", "Sets Fast to true");
    parser.addOption(setFast);

    parser.process(a);

    if (parser.isSet(idOption)) {
        auto id = parser.value(idOption).toInt();
        w.changePortById(id);
    }

    if (parser.isSet(setHeadless)) {
        w.hide();
        w.setIsGlEnabled(false);
    } else {
        w.show();
    }

    if (parser.isSet(setAtkfault))
        w.withGoalKick(true);
    if (parser.isSet(setFullSpeed))
        w.fullSpeed(true);
    if (parser.isSet(setFast))
        w.setFast();

    return QApplication::exec();
}
