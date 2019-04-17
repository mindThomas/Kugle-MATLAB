if (exist('time','var'))
	sim = [];
	sim.time = time;
	
	sim.position = X(:,1:2);
	sim.velocity = X(:,7:8);
	sim.q = X(:,3:6);		
	sim.dq = X(:,9:12);	

	sim.position_est = X_hat(:,1:2);	
	sim.velocity_est = X_hat(:,7:8);	
	sim.q_est = X_hat(:,3:6);	
	sim.dq_est = X_hat(:,9:12);
	
	sim.tau = tau;
	sim.q_ref = q_ref;
	sim.omega_b_ref = omega_b_ref;
	sim.S = S;
	
	clear time X X_hat tau q_ref omega_b_ref S;
end