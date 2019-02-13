#include "MyStrategy.h"
#include "Constants.h"
#include <string>

#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h"
#include "Simulation.h"
using namespace model;

MyStrategy::MyStrategy() { }

struct VisualPoint
{
    VisualPoint(double colorCode = 0){
        if (colorCode == 0)
        {r=1.0;g=0.0;b=0.0;}
        else
        {r=0.0;g=0.0;b=1.0;}
        a = 0.3;
    };
    double r;
    double g;
    double b;
    double a;

    virtual std::string toJSON() { return ""; };
};
struct Sphere: VisualPoint{
    Sphere(double x,double y,double z,double radius):VisualPoint(), x(x),y(y),z(z),radius(radius){};
    double x;
    double y;
    double z;
    double radius;

    std::string toJSON() override {
        return "{\"Sphere\":{\"x\":" +std::to_string(x)+",\"y\":"+std::to_string(y)+",\"z\":"+std::to_string(z)+
               ",\"radius\":"+std::to_string(radius)+",\"r\":"+std::to_string(r)+",\"g\":"+std::to_string(g)+
               ",\"b\":"+std::to_string(b)+",\"a\":"+std::to_string(a)+"}}";

    }
};
struct Line: VisualPoint{
    Line(double x1, double y1, double z1, double x2, double y2, double z2, double width, double colorCode = 0):VisualPoint(colorCode), x1(x1),y1(y1),z1(z1), x2(x2),y2(y2),z2(z2),width(width){};
    double x1;
    double y1;
    double z1;
    double x2;
    double y2;
    double z2;
    double width;

    std::string toJSON() override {
        return "{\"Line\":{\"x1\":" +std::to_string(x1)+",\"y1\":"+std::to_string(y1)+",\"z1\":"+std::to_string(z1)+
               ",\"x2\":" +std::to_string(x2)+",\"y2\":"+std::to_string(y2)+",\"z2\":"+std::to_string(z2)+
               ",\"width\":"+std::to_string(width)+",\"r\":"+std::to_string(r)+",\"g\":"+std::to_string(g)+
               ",\"b\":"+std::to_string(b)+",\"a\":"+std::to_string(a)+"}}";

    }
};


std::vector<Sphere> simulate_visList;
std::vector<Line> simulate_visList2;

std::string MyStrategy::custom_rendering() {
    /*return "";*/
    std::string res = "";
    if(simulate_visList.size()==0 || simulate_visList2.size()==0) return "";
    for(int i=0; i<simulate_visList.size();i++ )
    {
        if(i==0)
            res +=simulate_visList[i].toJSON();
        else
            res +=","+simulate_visList[i].toJSON();
    }

    for(int i=0; i<simulate_visList2.size();i++ )
    {
            res +=","+simulate_visList2[i].toJSON();
    }
    return "["+res+"]";
}

int currentTick = 0;
std::vector<Point3D> simulateBall;

void FillBallSimulate(const Game& game, const Rules& rules) {
    if (currentTick != game.current_tick) {
        currentTick = game.current_tick;
        simulateBall = {};
        simulate_visList = {};
        Simulation* sima = new Simulation(game, rules);
        for (int i = 0; i < 50; i++) {
            sima->update(0.02);
            simulateBall.emplace_back(sima->ball.position);
            simulate_visList.push_back(Sphere(sima->ball.position.x, sima->ball.position.y,sima->ball.position.z,game.ball.radius));
        }
        delete sima;
    }
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {
    FillBallSimulate(game,rules);
    Point3D ball(game.ball.x,game.ball.z,game.ball.y);
    //ball.set(game.ball.x, game.ball.z, game.ball.y);
    //ball_velocity.set(game.ball.velocity_x, game.ball.velocity_z, game.ball.velocity_y);

    // Наша стратегия умеет играть только на земле
    // Поэтому, если мы не касаемся земли, будет использовать нитро
    // чтобы как можно быстрее попасть обратно на землю
//    if ( !me.touch ) {
//        action.target_velocity_x = 0.0;
//        action.target_velocity_z = 0.0;
//        action.target_velocity_y = -MAX_ENTITY_SPEED;
//        action.jump_speed        = 0.0;
//        action.use_nitro         = true;
//        return;
//    }

    // Если при прыжке произойдет столкновение с мячом, и мы находимся
    // с той же стороны от мяча, что и наши ворота, прыгнем, тем самым
    // ударив по мячу сильнее в сторону противника
    bool jump = (   ball.distTo(me.x, me.z, me.y) < (BALL_RADIUS + ROBOT_MAX_RADIUS)
                    && me.y < game.ball.y );
    // Так как роботов несколько, определим нашу роль - защитник, или нападающий
    // Нападающим будем в том случае, если есть дружественный робот,
    // находящийся ближе к нашим воротам

   // simulate_visList = {};
    simulate_visList2 = {};
        bool is_attacker = false; // = (game.robots.size() == 2);
                    for (const Robot &robot : game.robots) {
                        if (   robot.is_teammate
                               && robot.id != me.id) {
                            if (robot.z < me.z) {
                                is_attacker = true;
                            }
                        }
                    }
    Point2D target_pos_for_vratar(00, -(rules.arena.depth / 2.0) - rules.arena.bottom_radius);
//    simulate_visList = {};
    //simulate_visList2 = {};
    Point2D vorotaVraga(0.0, rules.arena.depth / 2.0);
    Point2D ballc(game.ball.x, game.ball.z);
    //
        if (is_attacker) {
            //Simulation sima(game,rules);
            int j=1;
            for(const Point3D &point : simulateBall)
            {
                Point3D ball_pos = point;
                //simulate_visList.push_back(Sphere(ball_pos.x, ball_pos.y,ball_pos.z,game.ball.radius));
                if (ball_pos.z > me.z
                        ) {
                    // Посчитаем, с какой скоростью робот должен бежать,
                    // Чтобы прийти туда же, где будет мяч, в то же самое время
                    double t = j * 0.1;

                    Point2D currentBall(ball_pos.x, ball_pos.z);
                    Point2D res(vorotaVraga.x - currentBall.x, vorotaVraga.z - currentBall.z);
                    res = res.normalize() * BALL_RADIUS;
                    res = currentBall - res;
                    Point2D delta_pos(res.x - me.x, res.z - me.z);

                    double delta_pos_dist = delta_pos.dist();
                    double need_speed = delta_pos_dist / t;
                    // Если эта скорость лежит в допустимом отрезке
                    if (0.5 * ROBOT_MAX_GROUND_SPEED < need_speed
                        && need_speed < ROBOT_MAX_GROUND_SPEED) {
                        // То это и будет наше текущее действие
                        Point2D target_velocity(delta_pos.normalize(delta_pos_dist) * need_speed);
                        //simulate_visList2.push_back(Line(me.x, 0, me.z, target_velocity.x, 0, target_velocity.z, 2));
                        action.target_velocity_x = target_velocity.x;
                        action.target_velocity_z = target_velocity.z;
                        action.target_velocity_y = 0.0;
                        action.jump_speed = jump ? ROBOT_MAX_JUMP_SPEED : 0.0;
                        action.use_nitro = ball_pos.z > me.z && jump;
                        return;

                    }
                }
                j++;
            }
//            action.target_velocity_x = 0;
//            action.target_velocity_z = 0;
//            action.target_velocity_y = 0.0;
//            action.jump_speed = 0;
//            action.use_nitro = false;
//            return;
        }



    Point2D moiVorota(0.0, -rules.arena.depth / 2.0-rules.arena.goal_side_radius);
    Simulation sima(game,rules);
    bool isangerDetected = false;
    for(const Point3D &point : simulateBall) {
        Point3D ball_pos = point;
        //simulate_visList.push_back(Sphere(ball_pos.x, ball_pos.y,ball_pos.z,game.ball.radius));
        Point2D currentBall(ball_pos.x, ball_pos.z);
        Point2D res(moiVorota.x - currentBall.x, moiVorota.z - currentBall.z);
        res = res.normalize() * BALL_RADIUS;
        res = currentBall - res;
        Point2D delta_pos(res.x - me.x, res.z - me.z);
        if(ball_pos.z <-(rules.arena.depth / 2.0) + rules.arena.bottom_radius+BALL_RADIUS*2 && delta_pos.dist()<rules.arena.goal_width)//
        {
            target_pos_for_vratar.x = game.ball.x;
            target_pos_for_vratar.z = game.ball.z;
            isangerDetected = true;
            break;
        }
        }

        Point2D target_velocity(target_pos_for_vratar.x - me.x, target_pos_for_vratar.z - me.z);
        if (isangerDetected)
        {
            Point2D res(ballc.x , ballc.z - -(rules.arena.depth / 2.0+rules.arena.goal_depth+rules.arena.goal_side_radius));
            res = res.normalize() * BALL_RADIUS;
            res = ballc - res;
            Point2D delta_pos(res.x - me.x, res.z - me.z);
            target_velocity.x = delta_pos.x;
            target_velocity.z = delta_pos.z;
            simulate_visList2.push_back(Line(target_velocity.x, 0, target_velocity.z, me.x, 0, me.z, 10,0));
        }

        simulate_visList2.push_back(Line(target_velocity.x, 0, target_velocity.z, me.x, 0, me.z, 2,1));
        //target_velocity = target_velocity.normalize()*target_velocity.dist();//*=ROBOT_MAX_GROUND_SPEED;
        //if(ball.z<me.z+2*ROBOT_RADIUS)
            target_velocity*=ROBOT_MAX_GROUND_SPEED;
        //корректировать скорость и место прибытия
        //прыгать или нет?

        bool isJump = ball.distTo(me.x, me.z, me.y) < (2 * BALL_RADIUS + ROBOT_MAX_RADIUS) && me.y < game.ball.y;

        action.target_velocity_x = target_velocity.x;
        // TODO: прыгать прям чтобы по мячу попала
        action.target_velocity_z =target_velocity.z;
               // isJump && ball.y > 2.0 * ROBOT_MAX_RADIUS ? target_velocity.z + (BALL_RADIUS * 2) : target_velocity.z;
        action.target_velocity_y = 0.0;//0.5*ROBOT_MAX_GROUND_SPEED;
        action.jump_speed = isJump && game.ball.y > 2.0 * ROBOT_MAX_RADIUS ? ROBOT_MAX_JUMP_SPEED : 0.0;
        action.use_nitro = isJump;
        return;

}
