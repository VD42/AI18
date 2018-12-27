#include "MyStrategy.h"
#include "sym.h"

#include <unordered_map>

MyStrategy::MyStrategy()
#ifdef PRINT
	: writer(s)
#endif
{
}

void MyStrategy::act(model::Robot const& me, model::Rules const& rules, model::Game const& game, model::Action & action)
{
	static int rpt = 0;
	++rpt;

	static int forward = -1;
	static int goalkeeper = -1;

	static std::unordered_map<int, model::Action> actions;

	if (rpt == 1)
	{
		static int goals = 0;
		if (goals < game.players.front().score + game.players.back().score)
		{
			goals = game.players.front().score + game.players.back().score;
			forward = -1;
			goalkeeper = -1;
		}

		if (forward == -1 && goalkeeper == -1)
		{
			std::vector<std::pair<int, double>> robots;
			for (auto const& robot : game.robots)
				if (robot.is_teammate)
					robots.push_back(std::make_pair(robot.id, robot.z));
			std::sort(robots.begin(), robots.end(), [](std::pair<int, double> const& a, std::pair<int, double> const& b) { return a.second > b.second; });
			forward = robots.front().first;
			goalkeeper = robots.back().first;
		}

		static constexpr int ticks = 100;
		static std::vector<sym::Vec3D> ball_positions(ticks);

#ifdef PRINT
		auto add_key = [&](std::string key) {
			writer.Key(key.c_str(), key.length(), true);
		};

		s.Clear();
		writer.Reset(s);
		writer.StartArray();
#endif

		sym::init(rules);

		sym::ball.position = { game.ball.x, game.ball.y, game.ball.z };
		sym::ball.velocity = { game.ball.velocity_x, game.ball.velocity_y, game.ball.velocity_z };

		sym::robots = {};
		for (auto const& robot : game.robots)
		{
			sym::robots.emplace_back();
			sym::robots.back().arena_e = rules.ROBOT_ARENA_E;
			sym::robots.back().mass = rules.ROBOT_MASS;
			sym::robots.back().radius = robot.radius;
			sym::robots.back().radius_change_speed = 0.0;
			sym::robots.back().position = { robot.x, robot.y, robot.z };
			sym::robots.back().velocity = { robot.velocity_x, robot.velocity_y, robot.velocity_z };
			sym::robots.back().touch = robot.touch;
			sym::robots.back().touch_normal = { robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z };
			sym::robots.back().nitro = robot.nitro_amount;
		}

		for (int tick = 0; tick < ticks; ++tick)
		{
			sym::tick();
			ball_positions[tick] = sym::ball.position;

#ifdef PRINT
			writer.StartObject();
			add_key("Sphere");
			writer.StartObject();
			add_key("x");
			writer.Double(sym::ball.position.x);
			add_key("y");
			writer.Double(sym::ball.position.y);
			add_key("z");
			writer.Double(sym::ball.position.z);
			add_key("radius");
			writer.Double(0.5);
			add_key("r");
			writer.Double(1.0);
			add_key("g");
			writer.Double(1.0);
			add_key("b");
			writer.Double(1.0);
			add_key("a");
			writer.Double(0.5);
			writer.EndObject(8);
			writer.EndObject(1);
#endif
		}

		auto ball_3d = sym::Vec3D { game.ball.x, game.ball.y, game.ball.z };
		auto ball_2d = sym::Vec2D { game.ball.x, game.ball.z };

		{
			auto pos_3d = [&] () {
				for (auto const& robot : game.robots)
					if (robot.id == forward)
						return sym::Vec3D { robot.x, robot.y, robot.z };
				return sym::Vec3D();
			}();
			auto pos_2d = sym::Vec2D { pos_3d.x, pos_3d.z };
			bool stay = true;
			auto goal_2d = sym::Vec2D{ 0.0, rules.arena.depth / 2.0 };
			double t = 0.0;
			for (auto const& bp_3d : ball_positions)
			{
				t += 1.0 / (double)rules.TICKS_PER_SECOND;
				auto bp_2d = sym::Vec2D{ bp_3d.x, bp_3d.z };
				if (bp_3d.y > rules.ROBOT_MAX_RADIUS * 2.0 + game.ball.radius - std::numeric_limits<double>::epsilon())
					continue;
				auto best_pos = sym::normalize(goal_2d - bp_2d);
				best_pos.x = -best_pos.x;
				best_pos.y = -best_pos.y;
				best_pos = bp_2d + best_pos * (rules.ROBOT_MIN_RADIUS / 2.0 + game.ball.radius);
				auto vel = best_pos - pos_2d;
				auto distance = length(vel);
				auto speed = distance / t;
				if (0.0 < speed && speed <= rules.ROBOT_MAX_GROUND_SPEED)
				{
#ifdef PRINT
					writer.StartObject();
					add_key("Sphere");
					writer.StartObject();
					add_key("x");
					writer.Double(best_pos.x);
					add_key("y");
					writer.Double(rules.ROBOT_MIN_RADIUS);
					add_key("z");
					writer.Double(best_pos.y);
					add_key("radius");
					writer.Double(0.5);
					add_key("r");
					writer.Double(0.0);
					add_key("g");
					writer.Double(1.0);
					add_key("b");
					writer.Double(1.0);
					add_key("a");
					writer.Double(1.0);
					writer.EndObject(8);
					writer.EndObject(1);
#endif

					vel = sym::normalize(vel) * speed;
					actions[forward] = {};
					actions[forward].target_velocity_x = vel.x;
					actions[forward].target_velocity_y = 0.0;
					actions[forward].target_velocity_z = vel.y;

#ifdef PRINT
					writer.StartObject();
					add_key("Text");
					add_key(std::to_string(sym::length(ball_3d - pos_3d)));
					writer.EndObject(1);
#endif

					if (sym::length(ball_3d - pos_3d) < 2.0 * rules.ROBOT_MIN_RADIUS + game.ball.radius)
						actions[forward].jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
					stay = false;
					break;
				}
			}

			if (stay)
			{
				auto bp_2d = sym::Vec2D{ ball_positions.back().x, ball_positions.back().z };
				auto best_pos = sym::normalize(goal_2d - bp_2d);
				best_pos.x = -best_pos.x;
				best_pos.y = -best_pos.y;
				best_pos = bp_2d + best_pos * (rules.ROBOT_MIN_RADIUS / 2.0 + game.ball.radius);

#ifdef PRINT
				writer.StartObject();
				add_key("Sphere");
				writer.StartObject();
				add_key("x");
				writer.Double(best_pos.x);
				add_key("y");
				writer.Double(rules.ROBOT_MIN_RADIUS);
				add_key("z");
				writer.Double(best_pos.y);
				add_key("radius");
				writer.Double(0.5);
				add_key("r");
				writer.Double(0.0);
				add_key("g");
				writer.Double(1.0);
				add_key("b");
				writer.Double(1.0);
				add_key("a");
				writer.Double(0.5);
				writer.EndObject(8);
				writer.EndObject(1);
#endif

				auto vel = best_pos - pos_2d;
				vel = sym::normalize(vel) * rules.ROBOT_MAX_GROUND_SPEED;
				actions[forward] = {};
				actions[forward].target_velocity_x = vel.x;
				actions[forward].target_velocity_y = 0.0;
				actions[forward].target_velocity_z = vel.y;
			}
		}

		{
			auto pos_3d = [&] () {
				for (auto const& robot : game.robots)
					if (robot.id == goalkeeper)
						return sym::Vec3D{ robot.x, robot.y, robot.z };
				return sym::Vec3D();
			}();
			auto pos_2d = sym::Vec2D{ pos_3d.x, pos_3d.z };
			bool stay = true;
			auto goal_2d = sym::Vec2D{ 0.0, -rules.arena.depth / 2.0 };
			double t = 0.0;
			for (auto const& bp_3d : ball_positions)
			{
				t += 1.0 / (double)rules.TICKS_PER_SECOND;
				auto bp_2d = sym::Vec2D{ bp_3d.x, bp_3d.z };
				if (length(bp_2d - goal_2d) > rules.arena.depth / 3.0)
					continue;
				if (bp_3d.y > rules.ROBOT_MAX_RADIUS * 2.0 + game.ball.radius - std::numeric_limits<double>::epsilon())
					continue;
				auto best_pos = sym::normalize(goal_2d - bp_2d);
				best_pos = bp_2d + best_pos * (rules.ROBOT_MIN_RADIUS / 2.0 + game.ball.radius);
				auto vel = bp_2d - pos_2d;
				auto distance = length(vel);
				auto speed = distance / t;
				if (0.0 < speed && speed <= rules.ROBOT_MAX_GROUND_SPEED)
				{
#ifdef PRINT
					writer.StartObject();
					add_key("Sphere");
					writer.StartObject();
					add_key("x");
					writer.Double(best_pos.x);
					add_key("y");
					writer.Double(rules.ROBOT_MIN_RADIUS);
					add_key("z");
					writer.Double(best_pos.y);
					add_key("radius");
					writer.Double(0.5);
					add_key("r");
					writer.Double(1.0);
					add_key("g");
					writer.Double(1.0);
					add_key("b");
					writer.Double(0.0);
					add_key("a");
					writer.Double(1.0);
					writer.EndObject(8);
					writer.EndObject(1);
#endif

					vel = sym::normalize(vel) * speed;
					actions[goalkeeper] = {};
					actions[goalkeeper].target_velocity_x = vel.x;
					actions[goalkeeper].target_velocity_y = 0.0;
					actions[goalkeeper].target_velocity_z = vel.y;
					if (sym::length(ball_3d - pos_3d) < 2.0 * rules.ROBOT_MIN_RADIUS + game.ball.radius)
						actions[goalkeeper].jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
					stay = false;
					break;
				}
			}

			if (stay)
			{
#ifdef PRINT
				writer.StartObject();
				add_key("Sphere");
				writer.StartObject();
				add_key("x");
				writer.Double(goal_2d.x);
				add_key("y");
				writer.Double(rules.ROBOT_MIN_RADIUS);
				add_key("z");
				writer.Double(goal_2d.y);
				add_key("radius");
				writer.Double(0.5);
				add_key("r");
				writer.Double(1.0);
				add_key("g");
				writer.Double(1.0);
				add_key("b");
				writer.Double(0.0);
				add_key("a");
				writer.Double(0.5);
				writer.EndObject(8);
				writer.EndObject(1);
#endif

				auto vel = goal_2d - pos_2d;
				vel = sym::normalize(vel) * std::min(rules.ROBOT_MAX_GROUND_SPEED, length(vel) * (double)rules.TICKS_PER_SECOND);
				actions[goalkeeper] = {};
				actions[goalkeeper].target_velocity_x = vel.x;
				actions[goalkeeper].target_velocity_y = 0.0;
				actions[goalkeeper].target_velocity_z = vel.y;
			}
		}

	#ifdef PRINT
		writer.EndArray(ticks);
	#endif

	}

	if (rpt == game.robots.size() / 2)
		rpt = 0;

	if (me.id == forward)
		action = actions[forward];

	if (me.id == goalkeeper)
		action = actions[goalkeeper];
}

#ifdef PRINT
std::string MyStrategy::custom_rendering()
{
	return s.GetString();
}
#endif
