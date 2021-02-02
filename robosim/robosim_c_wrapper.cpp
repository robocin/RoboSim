#include "../src/sslworld.h"

extern "C"
{
    SSLWorld *newWorld(int fieldType, int nRobotsBlue, int nRobotsYellow, int timeStep_ms,
                    double *ballPos, double *blueRobotsPos, double *yellowRobotsPos)
    {
        return new SSLWorld(fieldType, nRobotsBlue, nRobotsYellow, timeStep_ms / 1000.0,
                         ballPos, blueRobotsPos, yellowRobotsPos);
    }
    void delWorld(SSLWorld *world) { delete world; }
    void step(SSLWorld *world, double *act)
    {
        std::vector<std::tuple<double, double, double, double, bool, double, double, bool>> actions;
        actions.clear();
        
        for (int i = 0; i < Config::Field().getRobotsCount(); i = i + 8)
        {
            std::tuple<double, double, double, double, bool, double, double, bool> action(act[i], act[i + 1], act[i + 2], act[i + 3], act[i + 4], act[i + 5], act[i + 6], act[i + 7]);
            actions.push_back(action);
        }
        world->step(world->getTimeStep(), actions);
    }
    void getState(SSLWorld *world, double *state_data)
    {
        const std::vector<double> state = world->getState();
        const double *state_features = state.data();
        memcpy(state_data, state_features, state.size() * sizeof(double));
    }
    void getFieldParams(SSLWorld *world, double *params_data)
    {
        const std::vector<double> params = world->getFieldParams();
        const double *params_features = params.data();
        memcpy(params_data, params_features, params.size() * sizeof(double));
    }
    int getEpisodeTime(SSLWorld *world) { return world->getEpisodeTime(); }
    int getGoalsBlue(SSLWorld *world) { return world->getGoals()[0]; }
    int getGoalsYellow(SSLWorld *world) { return world->getGoals()[1]; }
    void replace(SSLWorld *world, double *ball_data, double *pos_blue_data, double *pos_yellow_data)
    {
        world->replace(ball_data, pos_blue_data, pos_yellow_data);
    }
    void replace_with_vel(SSLWorld *world, double *ball_data, double *pos_blue_data, double *pos_yellow_data)
    {
        world->replace_with_vel(ball_data, pos_blue_data, pos_yellow_data);
    }
}
