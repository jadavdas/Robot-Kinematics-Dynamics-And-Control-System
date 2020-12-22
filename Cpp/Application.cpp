#include <iostream>
#include <eigen/Eigen/Dense>

#include "AdvancedRoboticsLibrary.h"
//#include "Robot.h"

# define M_PI	3.14159265358979323846  /* pi */

using namespace std;;

int main()
{
	// SCARA robot paramters

	Eigen::VectorXd thetalist(3);
	thetalist << 0.1, 0.1, 0.1;

	Eigen::VectorXd dthetalist(3);
	dthetalist << 0.1, 0.2, 0.3;

	Eigen::VectorXd g(3);
	g << 0, 0, -9.8;

	std::vector<Eigen::MatrixXd> Mlist;
	std::vector<Eigen::MatrixXd> Glist;

	Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0.089159,
		0, 0, 0, 1;

	Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
		0, 1, 0, 0.13585,
		-1, 0, 0, 0,
		0, 0, 0, 1;

	Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
		0, 1, 0, -0.1197,
		0, 0, 1, 0.395,
		0, 0, 0, 1;

	Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0.14225,
		0, 0, 0, 1;

	Mlist.push_back(M01);
	Mlist.push_back(M12);
	Mlist.push_back(M23);
	Mlist.push_back(M34);

	Eigen::VectorXd G1(6);
	G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;

	Eigen::VectorXd G2(6);
	G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;

	Eigen::VectorXd G3(6);
	G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

	Glist.push_back(G1.asDiagonal());
	Glist.push_back(G2.asDiagonal());
	Glist.push_back(G3.asDiagonal());

	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1, 0, 1, 0,
		0, 1, 0, -0.089, 0, 0,
		0, 1, 0, -0.089, 0, 0.425;

	Eigen::MatrixXd Slist = SlistT.transpose();

	double dt = 0.01;
	Eigen::VectorXd thetaend(3);
	thetaend << M_PI / 2, M_PI / 2, M_PI / 2;
	double Tf = 1.0;
	int N = int(1.0 * Tf / dt);
	int method = 5;

	Eigen::MatrixXd traj = roblib::utils::JointTrajectory(thetalist, thetaend, Tf, N, method);
	Eigen::MatrixXd thetamatd = traj;
	Eigen::MatrixXd dthetamatd = Eigen::MatrixXd::Zero(N, 3);
	Eigen::MatrixXd ddthetamatd = Eigen::MatrixXd::Zero(N, 3);
	dt = Tf / (N - 1.0);
	for (int i = 0; i < N - 1; ++i) 
	{
		dthetamatd.row(i + 1) = (thetamatd.row(i + 1) - thetamatd.row(i)) / dt;
		ddthetamatd.row(i + 1) = (dthetamatd.row(i + 1) - dthetamatd.row(i)) / dt;
	}

	Eigen::VectorXd gtilde(3);
	gtilde << 0.8, 0.2, -8.8;

	std::vector<Eigen::MatrixXd> Mtildelist;
	std::vector<Eigen::MatrixXd> Gtildelist;

	Eigen::Matrix4d Mhat01;
	Mhat01 << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0.1,
		0, 0, 0, 1;

	Eigen::Matrix4d Mhat12;
	Mhat12 << 0, 0, 1, 0.3,
		0, 1, 0, 0.2,
		-1, 0, 0, 0,
		0, 0, 0, 1;

	Eigen::Matrix4d Mhat23;
	Mhat23 << 1, 0, 0, 0,
		0, 1, 0, -0.2,
		0, 0, 1, 0.4,
		0, 0, 0, 1;

	Eigen::Matrix4d Mhat34;
	Mhat34 << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0.2,
		0, 0, 0, 1;

	Mtildelist.push_back(Mhat01);
	Mtildelist.push_back(Mhat12);
	Mtildelist.push_back(Mhat23);
	Mtildelist.push_back(Mhat34);

	Eigen::VectorXd Ghat1(6);
	Ghat1 << 0.1, 0.1, 0.1, 4, 4, 4;

	Eigen::VectorXd Ghat2(6);
	Ghat2 << 0.3, 0.3, 0.1, 9, 9, 9;

	Eigen::VectorXd Ghat3(6);
	Ghat3 << 0.1, 0.1, 0.1, 3, 3, 3;

	Gtildelist.push_back(Ghat1.asDiagonal());
	Gtildelist.push_back(Ghat2.asDiagonal());
	Gtildelist.push_back(Ghat3.asDiagonal());

	Eigen::MatrixXd Ftipmat = Eigen::MatrixXd::Ones(N, 6);
	double Kp = 20.0;
	double Ki = 10.0;
	double Kd = 18.0;
	int intRes = 8;

	int numTest = 3;  // test 0, N/2-1, N-1 indices of results
	Eigen::MatrixXd result_taumat(numTest, 3);
	Eigen::MatrixXd result_thetamat(numTest, 3);

	Eigen::VectorXd tau_timestep_beg(3);
	tau_timestep_beg << -14.2640765, -54.06797429, -11.265448;

	Eigen::VectorXd tau_timestep_mid(3);
	tau_timestep_mid << 31.98269367, 9.89625811, 1.47810165;

	Eigen::VectorXd tau_timestep_end(3);
	tau_timestep_end << 57.04391384, 4.75360586, -1.66561523;

	result_taumat << tau_timestep_beg.transpose(),
		tau_timestep_mid.transpose(),
		tau_timestep_end.transpose();

	Eigen::VectorXd theta_timestep_beg(3);
	theta_timestep_beg << 0.10092029, 0.10190511, 0.10160667;

	Eigen::VectorXd theta_timestep_mid(3);
	theta_timestep_mid << 0.85794085, 1.55124503, 2.80130978;

	Eigen::VectorXd theta_timestep_end(3);
	theta_timestep_end << 1.56344023, 3.07994906, 4.52269971;

	result_thetamat << theta_timestep_beg.transpose(),
		theta_timestep_mid.transpose(),
		theta_timestep_end.transpose();

	std::vector<Eigen::MatrixXd> controlTraj = roblib::utils::SimulateControl(thetalist, dthetalist, g, Ftipmat, Mlist, Glist, Slist, thetamatd, dthetamatd,
		ddthetamatd, gtilde, Mtildelist, Gtildelist, Kp, Ki, Kd, dt, intRes);

	Eigen::MatrixXd traj_tau = controlTraj.at(0);
	Eigen::MatrixXd traj_theta = controlTraj.at(1);
	Eigen::MatrixXd traj_tau_timestep(numTest, 3);

	traj_tau_timestep << traj_tau.row(0),
		traj_tau.row(int(N / 2) - 1),
		traj_tau.row(N - 1);

	Eigen::MatrixXd traj_theta_timestep(numTest, 3);
	traj_theta_timestep << traj_theta.row(0),
		traj_theta.row(int(N / 2) - 1),
		traj_theta.row(N - 1);

	cout << traj_tau_timestep << endl;
	cout << "Success !" << endl;
	cin.get();

}