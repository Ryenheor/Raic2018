//
// Created by badru on 19.12.2018.
//

#include "Point3D.h"
#include "Constants.h"
#include "model/Game.h"
#include "model/Rules.h"
#include "model/Action.h"
#include <vector>

using namespace model;
using namespace std;

inline double mymax(double val1, double val2)
{
    return val1>=val2? val1:val2;
}

inline double mymin(double val1, double val2)
{
    return val1<=val2? val1:val2;
}

inline double myabs(double val)
{
    return val>0? val:-val;

}

inline Point3D truncate(Point3D& vector, double max_length)
{
    if (max_length - vector.dist() >= EPS)
        return vector;

    return  vector*(max_length / vector.dist());
}

inline Point3D myclamp(Point3D& vector, double min_length, double max_length)
{
    return vector.dist() > max_length ? truncate(vector, max_length) :  vector.normalize()* min_length;
}

inline double myclamp(double x, double a, double b) { return mymax(a, mymin(x, b));}

// ограничить длину вектора до заданной длины
inline Point3D myclamp(Point3D vector, double len)
{
    return vector.dist() > len ? vector.normalize()*len : vector;
}

struct EntityAction{
public:
    EntityAction() {}
    EntityAction(Action action) {
        target_velocity.set(action.target_velocity_x, action.target_velocity_z, action.target_velocity_y);
        jump_speed = action.jump_speed;
        use_nitro = action.use_nitro;
    }

    Point3D target_velocity;
    bool use_nitro;
    double jump_speed;
};

struct Entity{
public:
    Entity() {}
    Entity(const Robot& robot) {
        position.set(robot.x,robot.z,robot.y);
        velocity.set(robot.velocity_x,robot.velocity_z,robot.velocity_y);
        touch_normal.set(robot.touch_normal_x,robot.touch_normal_z,robot.touch_normal_y);
        radius = robot.radius;
        mass = ROBOT_MASS;
        arena_e = ROBOT_ARENA_E;
        id = robot.id;
        touch = robot.touch;
        nitro = robot.nitro_amount;
        radius_change_speed = 0;
        is_teammate = robot.is_teammate;
    }
    Entity(const Robot& robot,EntityAction action_item) {
        position.set(robot.x,robot.z,robot.y);
        velocity.set(robot.velocity_x,robot.velocity_z,robot.velocity_y);
        touch_normal.set(robot.touch_normal_x,robot.touch_normal_z,robot.touch_normal_y);
        radius = robot.radius;
        mass = ROBOT_MASS;
        arena_e = ROBOT_ARENA_E;
        id = robot.id;
        touch = robot.touch;
        nitro = robot.nitro_amount;
        action = action_item;
        radius_change_speed = action_item.jump_speed;
        is_teammate = robot.is_teammate;
    }
    Entity(const Ball& ball) {
        position.set(ball.x,ball.z,ball.y);
        velocity.set(ball.velocity_x,ball.velocity_z,ball.velocity_y);
        radius = ball.radius;
        mass = BALL_MASS;
        arena_e = BALL_ARENA_E;
        id = 0;
    }
    Entity(double x, double z, double y,double r) { position.set(x,z,y); radius = r;}
    Entity(Point3D position, Point3D velocity,double radius,double mass):position(position),velocity(velocity),radius(radius),mass(mass)  {}
    void setPosition(double x_, double z_, double y_) { position.set(x_,z_,y_); }
    void setVelocity(double x_, double z_, double y_) { velocity.set(x_,z_,y_); }
    Point3D position;
    Point3D velocity;
    Point3D touch_normal;
    bool is_teammate;
    double radius;
    double mass;
    double arena_e;
    int id;
    bool touch;
    double nitro;
    double radius_change_speed;
    //TODO: Или забить?
    EntityAction action;
};


struct Dan {
public:
    Dan(double _distance,Point3D _normal): distance(_distance), normal(_normal){}
    double distance;
    Point3D normal;
};

Dan mymin(Dan val_1, Dan val_2)
{
    return val_1.distance<=val_2.distance? val_1: val_2;
}

double get_random(double min, double max) {
    /* Returns a random double between min and max */
    return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
}

struct Simulation{

public:
    Simulation(const Game& game, const Rules& rules): game(game), rules(rules){
        // перегнать роботов в моих роботов
        //robots = game.robots;
        // TODO: переписать класс робота снаследовать с Entity

        robots = {};
        for (const Robot &robot : game.robots) {
            robots.emplace_back(Entity(robot));
        }
        ball = Entity(game.ball);
        nitro_packs = game.nitro_packs;
        // стащить vect3d
    };
    ~Simulation(){
    }
    void collide_entities(Entity& a , Entity& b);
    Point3D* collide_with_arena(Entity& e);
    void update(double delta_time);
    void update(double delta_time,bool is_my_team, int myrobotid, Action action);
    void move(Entity& e, double delta_time);
    void tick();
    Dan dan_to_arena(Point3D point);
    Dan dan_to_arena_quarter(Point3D point);
    Dan dan_to_plane(Point3D point,Point3D point_on_plane,Point3D plane_normal);
    Dan dan_to_sphere_inner(Point3D point, Point3D sphere_center, double sphere_radius);
    Dan dan_to_sphere_outer(Point3D point,Point3D  sphere_center,double sphere_radius);

   // СonvergenceSim checkBallSimulaion(const model::Ball ball1);

    const Rules& rules;
    const Game& game;
    Entity ball;
    std::vector<Entity> robots;
    std::vector<NitroPack> nitro_packs;
//    Action& action;
};


//distance: dot(point - point_on_plane, plane_normal)
//скалярное произведение
double dot(const Point3D& first,const Point3D& second){
    return first.x*second.x+first.y*second.y+first.z*second.z;
}
void Simulation::collide_entities(Entity& a , Entity& b){
    Point3D delta_position = b.position - a.position;
    double distance = delta_position.dist();//length(delta_position)
    double penetration = a.radius + b.radius - distance;
    if (penetration > 0){
        double k_a = (1 / a.mass) / ((1 / a.mass) + (1 / b.mass));
        double k_b = (1 / b.mass) / ((1 / a.mass) + (1 / b.mass));
        Point3D normal = delta_position.normalize();
        a.position -= normal * penetration * k_a;
        b.position += normal * penetration * k_b;
        double delta_velocity = dot(b.velocity - a.velocity, normal)- b.radius_change_speed - a.radius_change_speed;
        //radius_change_speed = robot.action.jump_speed
        if (delta_velocity < 0) {
            double temp = (1 + get_random(MIN_HIT_E, MAX_HIT_E)) * delta_velocity;
            Point3D impulse = normal*temp;
            a.velocity += impulse * k_a;
            b.velocity -= impulse * k_b;
        }
    }
}

void Simulation::move(Entity& e, double delta_time){
    e.velocity = myclamp(e.velocity, MAX_ENTITY_SPEED);
    e.position+=e.velocity*delta_time;
    e.position.y-=GRAVITY*delta_time*delta_time/2;
    e.velocity.y-=GRAVITY*delta_time;
}
//clamp(vector, len) - ограничить длину вектора до заданной длины
//clamp(x, a, b) = max(a, min(x, b))
//min
//radius_change_speed = action.jump_speed
//Если мяч - 0. Так вроде

Point3D* Simulation::collide_with_arena(Entity& e){
    Dan dan = dan_to_arena(e.position);
    double penetration = e.radius - dan.distance;
    if (penetration > 0){
        e.position += dan.normal*penetration;
        double velocity = dot(e.velocity, dan.normal);// - e.radius_change_speed
        if (velocity < 0) {
            e.velocity -=  dan.normal*(1 + e.arena_e)*velocity;
            //откуда взялисьe.arena_e и e.radius_change_speed? И что это за переменные?
            //из Rules т.е. e это мяч или робот, а соответвующее значения для мяча и робота смотри в Rules
            return &(dan.normal);
        }
    }
    return nullptr;
}

void Simulation::update(double delta_time){
   // shuffle(robots)
    for (Entity &robot : robots) {
        if (robot.touch){
            Point3D target_velocity = myclamp(robot.action.target_velocity,ROBOT_MAX_GROUND_SPEED);
            target_velocity -= robot.touch_normal * dot(robot.touch_normal, target_velocity);
            Point3D target_velocity_change = target_velocity - robot.velocity;
            if (target_velocity_change.dist() > 0) {
                double acceleration = ROBOT_ACCELERATION * mymax(0, robot.touch_normal.y);
                robot.velocity += myclamp(target_velocity_change.normalize() * acceleration * delta_time,target_velocity_change.dist());
            }
        }

        if (robot.action.use_nitro){
            Point3D target_velocity_change = myclamp(robot.action.target_velocity - robot.velocity,robot.nitro * NITRO_POINT_VELOCITY_CHANGE);
            if (target_velocity_change.dist() > 0){
                Point3D acceleration = target_velocity_change.normalize() * ROBOT_NITRO_ACCELERATION;
                Point3D velocity_change = myclamp(acceleration * delta_time, target_velocity_change.dist());
                robot.velocity += velocity_change;
                robot.nitro -= velocity_change.dist() / NITRO_POINT_VELOCITY_CHANGE;
            }
        }

        move(robot, delta_time);
        robot.radius = ROBOT_MIN_RADIUS + (ROBOT_MAX_RADIUS - ROBOT_MIN_RADIUS) * robot.action.jump_speed / ROBOT_MAX_JUMP_SPEED;
        robot.radius_change_speed = robot.action.jump_speed;
    }

    Simulation::move(ball, delta_time);

    for (Entity &robot1 : robots) {
        for (Entity &robot2 : robots) {
            collide_entities(robot1, robot2);
        }
    }

    for (Entity &robot : robots) {
        collide_entities(robot, ball);
        Point3D* collision_normal = collide_with_arena(robot);
        if (collision_normal== nullptr){
            robot.touch = false;
        }
        else {
            robot.touch = true;
            robot.touch_normal = *collision_normal;
        }
        //delete collision_normal;
    }

    Point3D* ball_normal = collide_with_arena(ball);

    if (myabs(ball.position.z) > rules.arena.depth / 2 + ball.radius)
    {
        //goal_scored()
    }
   // delete ball_normal;

//    for (int i=0;i<robots.size();i++){
//        if (robots[i].nitro == MAX_NITRO_AMOUNT)
//            continue;
//        for (int j=0; j<nitro_packs.size();j++){
//            if (!game.nitro_packs[j].alive)
//                continue;
//            Point3D nitroPos(nitro_packs[j].x,nitro_packs[j].z,nitro_packs[j].y);
//            if ((robots[i].position - nitroPos).dist() <= robots[i].radius + robots[i].radius) {
//                robots[i].nitro = MAX_NITRO_AMOUNT;
//                nitro_packs[j].alive = false;
//                nitro_packs[j].respawn_ticks = NITRO_RESPAWN_TICKS;
//            }
//        }
//    }
}

void Simulation::update(double delta_time,bool is_my_team, int myrobotid, Action action)
{
    EntityAction action_for_robot(action);
    for (Entity &robot1 : robots) {
        if (robot1.is_teammate==is_my_team && robot1.id==myrobotid)
        {
            robot1.action = action_for_robot;
        }
    }
    for (Entity &robot : robots) {
        if (robot.touch){
            Point3D target_velocity = myclamp(robot.action.target_velocity,ROBOT_MAX_GROUND_SPEED);
            target_velocity -= robot.touch_normal * dot(robot.touch_normal, target_velocity);
            Point3D target_velocity_change = target_velocity - robot.velocity;
            if (target_velocity_change.dist() > 0) {
                double acceleration = ROBOT_ACCELERATION * mymax(0, robot.touch_normal.y);
                robot.velocity += myclamp(target_velocity_change.normalize() * acceleration * delta_time,target_velocity_change.dist());
            }
        }

        if (robot.action.use_nitro){
            Point3D target_velocity_change = myclamp(robot.action.target_velocity - robot.velocity,robot.nitro * NITRO_POINT_VELOCITY_CHANGE);
            if (target_velocity_change.dist() > 0){
                Point3D acceleration = target_velocity_change.normalize() * ROBOT_NITRO_ACCELERATION;
                Point3D velocity_change = myclamp(acceleration * delta_time, target_velocity_change.dist());
                robot.velocity += velocity_change;
                robot.nitro -= velocity_change.dist() / NITRO_POINT_VELOCITY_CHANGE;
            }
        }

        move(robot, delta_time);
        robot.radius = ROBOT_MIN_RADIUS + (ROBOT_MAX_RADIUS - ROBOT_MIN_RADIUS) * robot.action.jump_speed / ROBOT_MAX_JUMP_SPEED;
        robot.radius_change_speed = robot.action.jump_speed;
    }

    Simulation::move(ball, delta_time);

    for (Entity &robot1 : robots) {
        for (Entity &robot2 : robots) {
            collide_entities(robot1, robot2);
        }
    }

    for (Entity &robot : robots) {
        collide_entities(robot, ball);
        Point3D* collision_normal = collide_with_arena(robot);
        if (collision_normal== nullptr){
            robot.touch = false;
        }
        else {
            robot.touch = true;
            robot.touch_normal = *collision_normal;
        }
        //delete collision_normal;
    }

    Point3D* ball_normal = collide_with_arena(ball);

    if (myabs(ball.position.z) > rules.arena.depth / 2 + ball.radius)
    {
        //goal_scored()
    }
    // delete ball_normal;

//    for (int i=0;i<robots.size();i++){
//        if (robots[i].nitro == MAX_NITRO_AMOUNT)
//            continue;
//        for (int j=0; j<nitro_packs.size();j++){
//            if (!game.nitro_packs[j].alive)
//                continue;
//            Point3D nitroPos(nitro_packs[j].x,nitro_packs[j].z,nitro_packs[j].y);
//            if ((robots[i].position - nitroPos).dist() <= robots[i].radius + robots[i].radius) {
//                robots[i].nitro = MAX_NITRO_AMOUNT;
//                nitro_packs[j].alive = false;
//                nitro_packs[j].respawn_ticks = NITRO_RESPAWN_TICKS;
//            }
//        }
//    }

}

void Simulation::tick(){
    double delta_time = 1.0 / TICKS_PER_SECOND;
    for (int i=0; i<MICROTICKS_PER_TICK - 1;i++) {
        update(delta_time / MICROTICKS_PER_TICK);
    }

    for (NitroPack &nitro : nitro_packs)
    {
        if(nitro.alive)
            continue;
        nitro.respawn_ticks -= 1;
        if (nitro.respawn_ticks == 0)
            nitro.alive = true;
    }
}

//Определение ближайшей точки арены

Dan Simulation::dan_to_plane(Point3D point,Point3D point_on_plane,Point3D plane_normal) {
    return Dan(dot(point - point_on_plane, plane_normal),plane_normal);
}

Dan Simulation::dan_to_sphere_inner(Point3D point, Point3D sphere_center, double sphere_radius){
    return Dan(sphere_radius - (point - sphere_center).dist(),(sphere_center - point).normalize());
}

Dan Simulation::dan_to_sphere_outer(Point3D point,Point3D  sphere_center,double sphere_radius){
    return Dan((point - sphere_center).dist() - sphere_radius,(point - sphere_center).normalize());
}

Dan Simulation::dan_to_arena_quarter(Point3D point){
// Ground
    Dan dan = dan_to_plane(point, Point3D(0, 0, 0), Point3D(0, 0, 1));

// Ceiling
    dan = mymin(dan, dan_to_plane(point, Point3D(0, 0, rules.arena.height), Point3D(0, 0, -1)));

// Side x
    dan = mymin(dan, dan_to_plane(point, Point3D(rules.arena.width / 2.0, 0, 0), Point3D(-1, 0, 0)));

// Side z (goal)
    dan = mymin(dan, dan_to_plane(point, Point3D(0, (rules.arena.depth / 2.0) + rules.arena.goal_depth, 0),  Point3D(0, -1, 0)));

// Side z
//TODO Point3d ?
    Point2D v = Point2D(point.x, point.z) - ((rules.arena.goal_width / 2.0) - rules.arena.goal_top_radius, rules.arena.goal_height - rules.arena.goal_top_radius);
    if (point.x >= (rules.arena.goal_width / 2.0) + rules.arena.goal_side_radius
                    || point.y >= rules.arena.goal_height + rules.arena.goal_side_radius
                    || (
                    v.x > 0
                    && v.z > 0
                    && v.dist() >= rules.arena.goal_top_radius + rules.arena.goal_side_radius))
    {
        dan = mymin(dan, dan_to_plane(point, Point3D(0, rules.arena.depth / 2.0, 0), Point3D(0, -1, 0)));
    }

// Side x & ceiling (goal)
    if (point.z >= (rules.arena.depth / 2.0) + rules.arena.goal_side_radius)
    {
    // x
        dan = mymin(dan, dan_to_plane(point,Point3D(rules.arena.goal_width / 2.0, 0, 0),Point3D(-1, 0, 0)));
    // y
        dan = mymin(dan, dan_to_plane(point, Point3D(0, 0, rules.arena.goal_height), Point3D(0, 0, -1)));
//TODO уиииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииииии
    // Goal back corners
    //assert arena.bottom_radius == arena.goal_top_radius
    if (point.z > (rules.arena.depth / 2) + rules.arena.goal_depth - rules.arena.bottom_radius){
        dan = mymin(dan, dan_to_sphere_inner(
                point,
                Point3D(
                        myclamp(
                                point.x,
                                rules.arena.bottom_radius - (rules.arena.goal_width / 2.0),
                                (rules.arena.goal_width / 2.0) - rules.arena.bottom_radius
                        ),(rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.bottom_radius,
                        myclamp(
                                        point.y,
                                        rules.arena.bottom_radius,
                                        rules.arena.goal_height - rules.arena.goal_top_radius
                                )
                                ),
                rules.arena.bottom_radius));
        }
    }
    // Corner
    if (point.x > (rules.arena.width / 2.0) -  rules.arena.corner_radius
            && point.z > ( rules.arena.depth / 2.0) -  rules.arena.corner_radius)
    {
        dan = mymin(dan, dan_to_sphere_inner(
                point,
                Point3D(
                        ( rules.arena.width / 2.0) -  rules.arena.corner_radius,( rules.arena.depth / 2.0) -  rules.arena.corner_radius,
                                point.y

                ),
                rules.arena.corner_radius));
    }
    // Goal outer corner
    if (point.z < ( rules.arena.depth / 2.0) +  rules.arena.goal_side_radius)
    {
        // Side x
        if (point.x < ( rules.arena.goal_width / 2.0) +  rules.arena.goal_side_radius)
        {
            dan = mymin(dan, dan_to_sphere_outer(
                point,
                (
                        Point3D( rules.arena.goal_width / 2.0 +  rules.arena.goal_side_radius,
                                ( rules.arena.depth / 2.0) +  rules.arena.goal_side_radius,
                                point.y

                        )
                ),
                rules.arena.goal_side_radius));
        }
        // Ceiling
        if (point.y <  rules.arena.goal_height +  rules.arena.goal_side_radius){
            dan = mymin(dan, dan_to_sphere_outer(
                point,
                (
                        Point3D( point.x,( rules.arena.depth / 2.0) +  rules.arena.goal_side_radius,
                                rules.arena.goal_height +  rules.arena.goal_side_radius

                        )
                ),
                rules.arena.goal_side_radius));
        }
        // Top corner
        // TODO: check xyz variations
        Point2D o((rules.arena.goal_width / 2.0) - rules.arena.goal_top_radius, rules.arena.goal_height - rules.arena.goal_top_radius);
        Point2D v = Point2D(point.x, point.z) - o;
        if (v.x > 0 and v.z > 0){
            Point2D o = o + v.normalize() * (rules.arena.goal_top_radius + rules.arena.goal_side_radius);
            dan = mymin(dan, dan_to_sphere_outer(
                    point,
                    Point3D(o.x, (rules.arena.depth / 2.0) + rules.arena.goal_side_radius, o.z),
                    rules.arena.goal_side_radius));
        }
    }
    // Goal inside top corners
    if (point.z > (rules.arena.depth / 2.0) + rules.arena.goal_side_radius && point.y > rules.arena.goal_height - rules.arena.goal_top_radius)
    {
        // Side x
        if (point.x > (rules.arena.goal_width / 2.0) - rules.arena.goal_top_radius) {
            dan = mymin(dan, dan_to_sphere_inner(
                    point,
                    Point3D(
                            (rules.arena.goal_width / 2.0) - rules.arena.goal_top_radius,point.z,
                            rules.arena.goal_height - rules.arena.goal_top_radius

                    ),
                    rules.arena.goal_top_radius));
        }
        // Side z
        if (point.z > (rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.goal_top_radius)
        {
            dan = mymin(dan, dan_to_sphere_inner(
                    point,
                    Point3D(
                            point.x,(rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.goal_top_radius,
                            rules.arena.goal_height - rules.arena.goal_top_radius

                    ),
                    rules.arena.goal_top_radius));
        }
    }
    // Bottom corners
    if (point.y < rules.arena.bottom_radius)
    {
        // Side x
        if (point.x > (rules.arena.width / 2.0) - rules.arena.bottom_radius)
        {
            dan = mymin(dan, dan_to_sphere_inner(
            point,
            Point3D(
                    (rules.arena.width / 2) - rules.arena.bottom_radius,point.z,
                    rules.arena.bottom_radius

            ),
            rules.arena.bottom_radius));
        }
        // Side z
        if (point.z > (rules.arena.depth / 2.0) - rules.arena.bottom_radius
                && point.x >= (rules.arena.goal_width / 2.0) + rules.arena.goal_side_radius){
            dan = mymin(dan, dan_to_sphere_inner(
                point,
                Point3D(
                        point.x,(rules.arena.depth / 2.0) - rules.arena.bottom_radius,
                        rules.arena.bottom_radius

                ),
                rules.arena.bottom_radius));
        }
        // Side z (goal)
        if (point.z > (rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.bottom_radius)
        {
            dan = mymin(dan, dan_to_sphere_inner(
                point,
                Point3D(
                        point.x,(rules.arena.depth / 2.0) + rules.arena.goal_depth - rules.arena.bottom_radius,
                        rules.arena.bottom_radius

                ),
                rules.arena.bottom_radius));}
        // Goal outer corner
        //TODO
        Point2D o (
                (rules.arena.goal_width / 2.0) + rules.arena.goal_side_radius,
                        (rules.arena.depth / 2.0) + rules.arena.goal_side_radius
        );
        Point2D v = Point2D(point.x, point.z) - o;
        if (v.x < 0 && v.z < 0
            && v.dist() < rules.arena.goal_side_radius + rules.arena.bottom_radius){
            Point2D o = o + v.normalize() * (rules.arena.goal_side_radius + rules.arena.bottom_radius);
            dan = mymin(dan, dan_to_sphere_inner(
                point,
                Point3D(o.x,  o.z,rules.arena.bottom_radius),
                rules.arena.bottom_radius));
        }
        // Side x (goal)
        if (point.z >= (rules.arena.depth / 2.0) + rules.arena.goal_side_radius
                && point.x > (rules.arena.goal_width / 2.0) - rules.arena.bottom_radius){
        dan = mymin(dan, dan_to_sphere_inner(
                point,
                Point3D(
                        (rules.arena.goal_width / 2.0) - rules.arena.bottom_radius,
                                point.z,rules.arena.bottom_radius
                ),
                rules.arena.bottom_radius));
        }
        // Corner
        if (point.x > (rules.arena.width / 2.0) - rules.arena.corner_radius
                && point.z > (rules.arena.depth / 2.0) - rules.arena.corner_radius){
            //TODO
        Point2D corner_o = Point2D(
                (rules.arena.width / 2.0) - rules.arena.corner_radius,
                        (rules.arena.depth / 2.0) - rules.arena.corner_radius
        );
            Point2D n = Point2D(point.x, point.z) - corner_o;
        double dist = n.dist();
        if (dist > rules.arena.corner_radius - rules.arena.bottom_radius){
            Point2D n2 = n* (1 / dist);
            //TODO:chech this moment in point2d
            Point2D o2 = corner_o + n2 * (rules.arena.corner_radius - rules.arena.bottom_radius);
            dan = mymin(dan, dan_to_sphere_inner(
                    point,
                    Point3D(o2.x,o2.z , rules.arena.bottom_radius),
                    rules.arena.bottom_radius));
            }
        }
    }
    // Ceiling corners
    if (point.y > rules.arena.height - rules.arena.top_radius){
        // Side x
        if (point.x > (rules.arena.width / 2.0) - rules.arena.top_radius){
            dan = mymin(dan, dan_to_sphere_inner(
                    point,
                    Point3D(
                            (rules.arena.width / 2.0) - rules.arena.top_radius,
                            point.z,
                                    rules.arena.height - rules.arena.top_radius
                    ),
                    rules.arena.top_radius));
        }
        // Side z
        if (point.z > (rules.arena.depth / 2.0) - rules.arena.top_radius){
            dan = mymin(dan, dan_to_sphere_inner(
                    point,
                    Point3D(
                            point.x,
                            (rules.arena.depth / 2.0) - rules.arena.top_radius,
                            rules.arena.height - rules.arena.top_radius

                    ),
            rules.arena.top_radius));
        }
        // Corner
        if (point.x > (rules.arena.width / 2.0) - rules.arena.corner_radius
                && point.z > (rules.arena.depth / 2.0) - rules.arena.corner_radius){
            Point2D corner_o = Point2D(
                    (rules.arena.width / 2.0) - rules.arena.corner_radius,
                            (rules.arena.depth / 2.0) - rules.arena.corner_radius
            );
            Point2D dv = Point2D(point.x, point.z) - corner_o;
            if (dv.dist() > rules.arena.corner_radius - rules.arena.top_radius){
                Point2D n = dv.normalize();
                Point2D o2 = corner_o + n * (rules.arena.corner_radius - rules.arena.top_radius);
                dan = mymin(dan, dan_to_sphere_inner(
                        point,
                        Point3D(o2.x,o2.z,  rules.arena.height - rules.arena.top_radius),
                        rules.arena.top_radius));
            }
        }
    }
    return dan;
}

Dan Simulation::dan_to_arena(Point3D point){
    bool negate_x = point.x < 0;
    bool negate_z = point.z < 0;
    if (negate_x){
        point.x = -point.x;
    }
    if (negate_z){
        point.z = -point.z;
    }
    Dan result = dan_to_arena_quarter(point);
    if (negate_x){
        result.normal.x = -result.normal.x;
    }
    if (negate_z){
        result.normal.z = -result.normal.z;
    }
    return result;
}
