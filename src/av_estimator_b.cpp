/**
 * @file av_estimator_b.cpp
 * Filter b class method implementation
 *
 * @author frits.kuipers <f.p.kuipers@student.utwente.nl>
 * @author Moses Bangura <moses.bangura@anu.edu.au, dnovichman@hotmail.com>
 */

#include "../include/av_estimator_b.h"


av_estimator_b::av_estimator_b()
{
	/* Initialize attitude vehicle uORB message. */
	memset(&att, 0, sizeof(att));

	/* Advertise vehicle attitude */
	att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* Initialize attitude vehicle uORB message. */
	memset(&vel, 0, sizeof(vel));

	/* Advertise vehicle velocity estimated */
	vel_pub = orb_advertise(ORB_ID(vehicle_velocity_est_inertial), &vel);

	/* Advertise control state */
	memset(&cont_state, 0, sizeof(cont_state));
	cont_state_pub = orb_advertise(ORB_ID(control_state), &cont_state);
}

void av_estimator_b::update(Vector3f &a, Vector3f &w, Vector3f &vbar, Vector3f &mu, av_estimator_params  attitude_params, Vector3f &vhat_in, Vector3f &what_in, bool filter_a_valid, float &dt)
{
	/* Omega is copied in the class because it has to be published later */
	this->omega = w;
	this->acc = a;

	/* Copy the parameters for easier use */
	k1 = attitude_params.att_vel_k1;
	k3 = attitude_params.att_vel_k3;
	k5 = attitude_params.att_vel_k5;
	k6 = attitude_params.att_vel_k6;
	k7 = attitude_params.att_vel_k7;
	k8 = attitude_params.att_vel_k8;
	//k1vicon = attitude_params.att_vel_k1vicon;
	k1vc = attitude_params.att_vel_k2;

	/* Calculate v error only once*/
	verror = vhat - vbar;

	udot = -k5 * g * verror.transpose() * Rhat.transpose() * e3;

	/* Integration step */
	u = udot * dt + u_prev;

	Rhat_dot = Rhat_prev * skew(omega - beta_w) - k3 * skew((g/u * Rhat * verror).cross(e3)) * Rhat;

	/* Add magnetometer innovation */
	if(k6)
	{
		Vector3f mu_hat = Rhat_prev.transpose() * mu_init;
		float temp = mu.cross(mu_hat).transpose() * X.transpose()*e3;
		Matrix3f delta3 = skew(temp * X.transpose() * e3);
		Rhat_dot += k6 * Rhat_prev * delta3;
	}


	/*if(viconTimestampPrev!=vicon.timestamp)
	{
		viconTimestampPrev = vicon.timestamp;

		AngleAxisf rollAngle(vicon.roll, Vector3f::UnitX());
		AngleAxisf pitchAngle(vicon.pitch, Vector3f::UnitY());
		AngleAxisf yawAngle(vicon.yaw, Vector3f::UnitZ());

		Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
		Matrix3f Rvicon = q.matrix().transpose();
		Rhat_dot -= k1vicon * skew(e1.cross(Rhat * Rvicon * e1)) * Rhat;
	} */

	/* Integration step */
	Rhat = Rhat_dot*dt + Rhat_prev;

	/* Capture NaN in Rhat */
	if (Rhat != Rhat)
	{
		Rhat = Matrix3f::Identity();
	}

	/* X */
	X = u*Rhat;

	Vector3f vhat_dot_vc;

	if (filter_a_valid) {
		vhat_dot_vc	= k1vc * (vhat_prev - Rhat_prev.transpose() * (vhat_in - what_in));
		//printf("filrer a valid\n");
	}

	else
		vhat_dot_vc	= Vector3f::Zero();

	vhat_dot = -skew(omega-beta_w) * vhat_prev + a - beta_a - k1 * verror + g * X.transpose() * e3 + vhat_dot_vc;

	/* Integration step */
	vhat = vhat_dot * dt + vhat_prev;

	/* Offsets */
	beta_a_dot = k1 * k7 * verror; 
	beta_a = beta_a_dot * dt + beta_a_prev;
	beta_w_dot = k3 * k8 * (g/u * Rhat_prev * verror).cross(e3);
	beta_w = beta_w_dot * dt + beta_w_prev;

	/* Orthonormalize matrices using gram schmidt, this is because of the euler integration */
	orthonormalize(Rhat);

	/* Set previous values */
	vhat_prev 	= vhat;
	u_prev 		= u;
	Rhat_prev 	= Rhat;
	beta_a_prev = beta_a;
	beta_w_prev = beta_w;

	scaling_u = u;

	beta_a_filterb = beta_a_prev;
}

void av_estimator_b::publish(float timestamp, Vector3f vhat_a, Vector3f what_vel, bool valid)
{	
	/* Set timestamp */
	att.timestamp = timestamp;

	/* Extract the roll, pitch, yaw of Rhat' (from inertial frame to body fixed frame)*/	
	att.roll 	= atan2f(Rhat(2,1), Rhat(2,2));	
	att.pitch 	= -asinf(Rhat(2,0));	
	att.yaw 	= atan2f(Rhat(1,0), Rhat(0,0));	

	/* Also publish the rotation rate */
	att.rollspeed 	= omega(0);
	att.pitchspeed 	= omega(1);
	att.yawspeed 	= omega(2);

	/* Quaternion from rotation matrix */
	Quaternionf q(Rhat);
	att.q[0] = q.w();
	att.q[1] = q.x();
	att.q[2] = q.y();
	att.q[3] = q.z();
	att.q_valid = true;

	/* Copy rotation matrix */
	float Rot_matrix[9];
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			Rot_matrix[i*3+j] = Rhat(i,j);
		}
	}

	memcpy(&att.R, Rot_matrix, sizeof(float)*9);
	att.R_valid = true;

	/* Publish */
	if (att_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
	} else {
		warnx("NaN in roll/pitch/yaw estimate!");
		orb_advertise(ORB_ID(vehicle_attitude), &att);
	}

	/* Rotate V from filter in {A} to {B}*/
	Vector3f vhat_a2b = Rhat.transpose() * vhat_a;

	/* Pub control stuff */
	cont_state.x_acc = acc(0);
	cont_state.y_acc = acc(1);
	cont_state.z_acc = acc(2);
	cont_state.x_vel = vhat_a(0);
	cont_state.y_vel = vhat_a(1);
	cont_state.z_vel = vhat_a(2);
	cont_state.roll_rate = omega(0);
	cont_state.pitch_rate = omega(1);
	cont_state.yaw_rate = omega(2);
	memcpy(&cont_state.q, att.q, sizeof(float)*4);
	if (cont_state_pub != nullptr) {
		orb_publish(ORB_ID(control_state), cont_state_pub, &cont_state);
	} else {
		orb_advertise(ORB_ID(control_state), &cont_state);
	}


	/* Publish velocities */
	vel.timestamp			= timestamp;
	vel.inertial_valid		= valid;
	vel.inertial_bvx		= vhat_a2b(0);
	vel.inertial_bvy		= vhat_a2b(1);
	vel.inertial_bvz		= vhat_a2b(2);
	vel.inertial_ivx		= vhat_a(0);
	vel.inertial_ivy		= vhat_a(1);
	vel.inertial_ivz		= vhat_a(2);

	vel.wind_ivx		= what_vel(0);
	vel.wind_ivy		= what_vel(1);
	vel.wind_ivz		= what_vel(2);

	vel.u			= scaling_u;

	strapdown_est_vhat(0) =  vhat(0);
	strapdown_est_vhat(1) =  vhat(1);
	strapdown_est_vhat(2) =  vhat(2);


	/* Publish */
	if (vel.inertial_valid)
	{
		if (vel_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_velocity_est_inertial), vel_pub, &vel);
		} else {
			orb_advertise(ORB_ID(vehicle_velocity_est_inertial), &vel);
		}
	}
}


