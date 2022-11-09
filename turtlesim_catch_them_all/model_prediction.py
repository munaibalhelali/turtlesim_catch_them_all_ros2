import numpy as np

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        self.goal = None

        self.x_obs = 5
        self.y_obs = 0.1

    def set_goal(self, goal):
        self.goal = goal 

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        v_t = v_t + dt * pedal - v_t/25.0
        
        x_t = x_t + dt * v_t * np.cos(psi_t)
        y_t = y_t + dt * v_t * np.sin(psi_t) 
        psi_t += dt * v_t *np.tan(steering)/2.5
        
        
        return [x_t, y_t, psi_t, v_t]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        
        for k in range(self.horizon):
            v_start = state[3]
            state = self.plant_model(state, self.dt, u[k*2], u[k*2+1])
            x_diff = abs(state[0] - ref[0])
            y_diff = abs(state[1] - ref[1])
            psi_diff = abs(state[2] - ref[2])

            obs_dist_x = abs(state[0] - self.x_obs)
            obs_dist_y = abs(state[1] - self.y_obs)

            obs_dist = np.sqrt(obs_dist_x**2 + obs_dist_y**2)


            cost += np.sqrt(x_diff**2+y_diff**2 + psi_diff**2 + (state[3] - v_start)**2)
            
            cost += 1/obs_dist**2*10

        speed_kph = state[3]*3.6
        if speed_kph > 10.0:
            cost += speed_kph * 100

        return cost

