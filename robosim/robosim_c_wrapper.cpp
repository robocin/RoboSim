#include "../src/world.h"

extern "C"
{
    World *newWorld(int fieldType, int nRobotsBlue, int nRobotsYellow, int timeStep_ms,
                    double *ballPos, double *blueRobotsPos, double *yellowRobotsPos)
    {
        return new World(fieldType, nRobotsBlue, nRobotsYellow, timeStep_ms / 1000.0,
                         ballPos, blueRobotsPos, yellowRobotsPos);
    }
    void delWorld(World *world) { delete world; }
    void step(World *world, double *act)
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
    void getState(World *world, double *state_data)
    {
        const std::vector<double> state = world->getState();
        const double *state_features = state.data();
        memcpy(state_data, state_features, state.size() * sizeof(double));
    }
    void getFieldParams(World *world, double *params_data)
    {
        const std::vector<double> params = world->getFieldParams();
        const double *params_features = params.data();
        memcpy(params_data, params_features, params.size() * sizeof(double));
    }
    int getEpisodeTime(World *world) { return world->getEpisodeTime(); }
    int getGoalsBlue(World *world) { return world->getGoals()[0]; }
    int getGoalsYellow(World *world) { return world->getGoals()[1]; }
    void replace(World *world, double *ball_data, double *pos_blue_data, double *pos_yellow_data)
    {
        world->replace(ball_data, pos_blue_data, pos_yellow_data);
    }
    void replace_with_vel(World *world, double *ball_data, double *pos_blue_data, double *pos_yellow_data)
    {
        world->replace_with_vel(ball_data, pos_blue_data, pos_yellow_data);
    }
}
