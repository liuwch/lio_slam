#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>

using namespace std;
using namespace gtsam;

int main()
{

  //　向量保存好模拟的位姿和测量，到时候一个个往isam2里填加
  std::vector< BetweenFactor<Pose2> > gra;
  std::vector< Pose2 > initPose;

  noiseModel::Diagonal::shared_ptr model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  gra.push_back(BetweenFactor<Pose2>(1, 2, Pose2(5, 0, 0     ), model));
  gra.push_back(BetweenFactor<Pose2>(2, 3, Pose2(5, 0, -M_PI_2), model));
  gra.push_back(BetweenFactor<Pose2>(3, 4, Pose2(5, 0, -M_PI_2), model));
  gra.push_back(BetweenFactor<Pose2>(4, 5, Pose2(5, 0, -M_PI_2), model));
  gra.push_back(BetweenFactor<Pose2>(5, 2, Pose2(5, 0, -M_PI_2), model));

  initPose.push_back(Pose2(0.5, -0.0,  0.2   ));
  initPose.push_back( Pose2(5.3, 0.1, -0.2   ));
  initPose.push_back( Pose2(10.1, -0.1,  -M_PI_2));
  initPose.push_back( Pose2(10.0, -5.0,  -M_PI  ));
  initPose.push_back( Pose2(5.1, -5.1, M_PI_2));

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  // 注意isam2的graph里只添加isam2更新状态以后新测量到的约束
  NonlinearFactorGraph graph;
  Values initialEstimate;

  // 第一个姿势不需要更新
  for( int i =0; i<5 ;i++)
  {
          // 添加对当前姿势的初步猜测
           initialEstimate.insert(i+1, initPose[i]);

           if(i == 0)
           {
               //  在第一个姿势上添加一个前置，将其设置为原点
               //一个先验因子由一个均值和一个噪声模型组成(协方差矩阵)
               noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
               graph.push_back(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

           }else
           {

               graph.push_back(gra[i-1]);  // Ie:当I = 1时，机器人在pos2，在pos1和pos2之间有一条边gra[0]
               if(i == 4)
               {
                   graph.push_back(gra[4]);  //  当机器人在pos5时，有两条边，一条是pos4 ->pos5，另一条是pos5->pos2 (grad[4])
               }
               isam.update(graph, initialEstimate);
               isam.update();

               Values currentEstimate = isam.calculateEstimate();
               cout << "****************************************************" << endl;
               cout << "Frame " << i << ": " << endl;
               currentEstimate.print("Current estimate: ");
               initialEstimate.print("Initial estimate: ");

               // 特别重要，update以后，清空原来的约束。已经加入到isam2的那些会用bayes tree保管，你不用操心了。
               graph.resize(0);
               initialEstimate.clear();
           }
  }
  return 0; 
}