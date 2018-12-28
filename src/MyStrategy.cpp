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
		auto add_string = [&] (std::string key) {
			writer.String(key.c_str(), key.length(), true);
		};

		auto print_sphere = [&] (sym::Vec3D const& position, double radius, double r, double g, double b, double a) {
			writer.StartObject();
			add_string("Sphere");
			writer.StartObject();
			add_string("x");
			writer.Double(position.x);
			add_string("y");
			writer.Double(position.y);
			add_string("z");
			writer.Double(position.z);
			add_string("radius");
			writer.Double(radius);
			add_string("r");
			writer.Double(r);
			add_string("g");
			writer.Double(g);
			add_string("b");
			writer.Double(b);
			add_string("a");
			writer.Double(a);
			writer.EndObject();
			writer.EndObject();
		};

		auto print_text = [&] (std::string text) {
			writer.StartObject();
			add_string("Text");
			add_string(text);
			writer.EndObject();
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
			print_sphere(sym::ball.position, 0.5, 1.0, 1.0, 1.0, 0.5);
#endif
		}

		auto sym_speed = [&rules] (sym::Vec2D const& begin, sym::Vec2D const& end, sym::Vec2D const& velocity, double time) {
			auto speed = length(end - begin) / time;
			std::optional<bool> grow = std::nullopt;
			while (0.0 <= speed && speed <= rules.ROBOT_MAX_GROUND_SPEED)
			{
				auto current_begin = begin;
				auto current_velocity = velocity;
				auto current_time = time;
				auto min_target_velocity_change_length = std::numeric_limits<double>::max();
				while (true)
				{
					auto delta_pos = end - current_begin;
					auto target_velocity = sym::normalize(delta_pos) * speed;
					auto target_velocity_change = target_velocity - current_velocity;
					auto target_velocity_change_length = sym::length(target_velocity_change);
					if (target_velocity_change_length < min_target_velocity_change_length)
						min_target_velocity_change_length = target_velocity_change_length;
					else
						break;
					if (target_velocity_change_length > 0.0)
					{
						current_velocity += sym::clamp(
							sym::normalize(target_velocity_change) * (rules.ROBOT_ACCELERATION / (double)(rules.TICKS_PER_SECOND * rules.MICROTICKS_PER_TICK)),
							sym::length(target_velocity_change)
						);
					}
					else
					{
						current_time -= length(delta_pos) / speed;
						break;
					}
					current_begin += current_velocity * (1.0 / (double)(rules.TICKS_PER_SECOND * rules.MICROTICKS_PER_TICK));
					current_time -= (1.0 / (double)(rules.TICKS_PER_SECOND * rules.MICROTICKS_PER_TICK));
				}
				if (std::abs(current_time) < (1.0 / (double)(rules.TICKS_PER_SECOND * rules.MICROTICKS_PER_TICK * 10)))
					break;
				if (current_time < 0.0)
				{
					if (!grow.has_value())
						grow = true;
					else if (!grow.value())
						break;
					speed += 0.01;
				}
				else
				{
					if (!grow.has_value())
						grow = false;
					else if (grow.value())
						break;
					speed -= 0.01;
				}
			}
			return sym::clamp(speed, 0.0, rules.ROBOT_MAX_GROUND_SPEED);
		};

		auto ball_3d = sym::Vec3D { game.ball.x, game.ball.y, game.ball.z };
		auto ball_2d = sym::Vec2D { game.ball.x, game.ball.z };

		{
			auto pos_3d = [&] () {
				for (auto const& robot : game.robots)
					if (robot.id == forward)
						return sym::Vec3D { robot.x, robot.y, robot.z };
				return sym::Vec3D();
			}();
			auto vel_3d = [&] () {
				for (auto const& robot : game.robots)
					if (robot.id == forward)
						return sym::Vec3D { robot.velocity_x, robot.velocity_y, robot.velocity_z };
				return sym::Vec3D();
			}();
			auto pos_2d = sym::Vec2D { pos_3d.x, pos_3d.z };
			auto vel_2d = sym::Vec2D { vel_3d.x, vel_3d.z };
			bool stay = true;
			auto goal_2d = sym::Vec2D { sym::clamp(
				ball_2d.x,
				-rules.arena.goal_width / 2.0 + game.ball.radius * 2.0 + std::numeric_limits<double>::epsilon(),
				rules.arena.goal_width / 2.0 - game.ball.radius * 2.0 - std::numeric_limits<double>::epsilon()
			), rules.arena.depth / 2.0 };
			double t = 0.0;
			for (auto const& bp_3d : ball_positions)
			{
				t += 1.0 / (double)rules.TICKS_PER_SECOND;
				if (bp_3d.y > rules.ROBOT_MIN_RADIUS * 2.0 + game.ball.radius - std::numeric_limits<double>::epsilon())
					continue;
				auto bp_2d = sym::Vec2D { bp_3d.x, bp_3d.z };
				auto best_pos = sym::normalize(goal_2d - bp_2d);
				best_pos.x = -best_pos.x;
				best_pos.y = -best_pos.y;
				best_pos = bp_2d + best_pos * (rules.ROBOT_MIN_RADIUS / 2.0 + game.ball.radius);
				auto vel = best_pos - pos_2d;
				auto speed = sym_speed(pos_2d, best_pos, vel_2d, t);

#ifdef PRINT
				print_sphere({ best_pos.x, rules.ROBOT_MIN_RADIUS, best_pos.y }, 0.5, 0.0, 1.0, 1.0, 1.0);
#endif

				vel = sym::normalize(vel) * speed;
				actions[forward] = {};
				actions[forward].target_velocity_x = vel.x;
				actions[forward].target_velocity_y = 0.0;
				actions[forward].target_velocity_z = vel.y;

				if (sym::length(ball_3d - pos_3d) < 2.0 * rules.ROBOT_MIN_RADIUS + game.ball.radius)
					actions[forward].jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
				stay = false;
				break;
			}

			if (stay)
			{
				auto bp_2d = sym::Vec2D{ ball_positions.back().x, ball_positions.back().z };
				auto best_pos = sym::normalize(goal_2d - bp_2d);
				best_pos.x = -best_pos.x;
				best_pos.y = -best_pos.y;
				best_pos = bp_2d + best_pos * (rules.ROBOT_MIN_RADIUS / 2.0 + game.ball.radius);

#ifdef PRINT
				print_sphere({ best_pos.x, rules.ROBOT_MIN_RADIUS, best_pos.y }, 0.5, 0.0, 1.0, 1.0, 0.5);
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
			auto vel_3d = [&] () {
				for (auto const& robot : game.robots)
					if (robot.id == forward)
						return sym::Vec3D { robot.velocity_x, robot.velocity_y, robot.velocity_z };
				return sym::Vec3D();
			}();
			auto pos_2d = sym::Vec2D { pos_3d.x, pos_3d.z };
			auto vel_2d = sym::Vec2D { vel_3d.x, vel_3d.z };
			bool stay = true;
			auto goal_2d = sym::Vec2D { sym::clamp(
				ball_2d.x,
				-rules.arena.goal_width / 2.0 + game.ball.radius * 2.0 + std::numeric_limits<double>::epsilon(),
				rules.arena.goal_width / 2.0 - game.ball.radius * 2.0 - std::numeric_limits<double>::epsilon()
			), -rules.arena.depth / 2.0 };
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
				auto speed = sym_speed(pos_2d, best_pos, vel_2d, t);
#ifdef PRINT
				print_sphere({ best_pos.x, rules.ROBOT_MIN_RADIUS, best_pos.y }, 0.5, 1.0, 1.0, 0.0, 1.0);
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

			if (stay)
			{
#ifdef PRINT
				print_sphere({ goal_2d.x, rules.ROBOT_MIN_RADIUS, goal_2d.y }, 0.5, 1.0, 1.0, 0.0, 0.5);
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
