// ICM_MotionPlanning.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//#include "RRT.h"
//#include "FormClosure.h"
#include "TaskSet.h"
#include "Visual.h"
#include "RRT.h"
#include "Problem.h"

#include <cassert>
#include <iostream>
#include <chrono>
#include <iomanip>

#pragma warning(disable : 4996)

int main(int argc, char* argv[])
{
    std::cout << "Welcome to Sensorless ICM planner!\n";
    std::cout << "Setting       -> 1" << std::endl;
    std::cout << "Generate Path(RRT) -> 2" << std::endl;
    std::cout << "Generate Path(Reverse RRT)  -> 3" << std::endl;
    int i = 0;
    std::cout << ">";   std::cin >> i;

    if (i == 1) {
        TaskSet setting;
        setting.run();
    }
    else if (i == 2) {
		Problem* p = new Problem(new RRT);
		p->pathplanning();

		delete p;

    }
	else if(i == 3){
		Problem* p = new Problem(new RevRRT);
		p->pathplanning();

		delete p;
	}
//    else if (i == 3) {
//        //Node fin(18.8, -21.2, -24.3, 18.9, -14.8, -3);
//        //Node fin(38.1, -39.2, -63.4, 25.7, -41.5, -41.4);
//        Node fin(-44.4, 49.9, 48.7, 64.4, -27.8, -62.6);
//        //Node fin(25, -30, -75, 40, -50, -30);
//        //Node fin(45.7, -37.2, 28.9, -11.5, 27.6, -55.4);
//
//        FormClosure fc(fin);
//        fc.close();
//        Node fcfin = fc.get_fcangle();
//        for (int i = 0; i < Node::dof; ++i) {
//            std::cout << fcfin[i] << ", ";
//        }
//        std::cout << std::endl;
//    }
//    else if (i == 4) {
//        /*Node open(90, 0, 0, 90, 0, 0);
//        FormClosure fc(open);
//        fc.close();
//        Node fcini = fc.get_fcangle();*/
//        Node fcini(32, -39.2, -90, 22.7, -41.5, -44.8);
//        RevRRT rrt(fcini);
//        rrt.planning();
//        rrt.pathprint_IO();
//    }
//    else if (i == 5) {
//        Node ini(25, -30, -75, 30, -30, -20);
//        Node fin(32, -39.2, -90, 22.7, -41.5, -44.8);
//        
//        RRTConnect rrt(ini, fin);
//        NodeList nl = rrt.planning();
//        nl.printIO();
//    }

}

// プログラムの実行: Ctrl + F5 または [デバッグ] > [デバッグなしで開始] メニュー
// プログラムのデバッグ: F5 または [デバッグ] > [デバッグの開始] メニュー

// 作業を開始するためのヒント: 
//    1. ソリューション エクスプローラー ウィンドウを使用してファイルを追加/管理します 
//   2. チーム エクスプローラー ウィンドウを使用してソース管理に接続します
//   3. 出力ウィンドウを使用して、ビルド出力とその他のメッセージを表示します
//   4. エラー一覧ウィンドウを使用してエラーを表示します
//   5. [プロジェクト] > [新しい項目の追加] と移動して新しいコード ファイルを作成するか、[プロジェクト] > [既存の項目の追加] と移動して既存のコード ファイルをプロジェクトに追加します
//   6. 後ほどこのプロジェクトを再び開く場合、[ファイル] > [開く] > [プロジェクト] と移動して .sln ファイルを選択します
