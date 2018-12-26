#include "MyStrategy.h"

#include <algorithm>
#include <cmath>

#include "sym.h"

namespace
{
	struct point
	{
		double x;
		double y;
		double z;

		double length()
		{
			return std::sqrt(x * x + y * y + z * z);
		}

		void norm()
		{
			auto d = length();
			x /= d;
			y /= d;
			z /= d;
		}

		void reverse()
		{
			x = -x;
			y = -y;
			z = -z;
		}

		point add(point const& point)
		{
			return {
				point.x + x,
				point.y + y,
				point.z + z,
			};
		}

		point sub(point const& point)
		{
			return {
				point.x - x,
				point.y - y,
				point.z - z,
			};
		}

		void mul(double scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
		}

		void print(std::string title)
		{
			printf("%s { %f, %f, %f } length %f\r\n", title.c_str(), x, y, z, length());
		}

		void drop_normal(point const& normal)
		{
			if (1.0 - std::numeric_limits<double>::epsilon() < std::abs(normal.x) && std::abs(normal.x) < 1.0 + std::numeric_limits<double>::epsilon())
				x = 0.0;
			if (1.0 - std::numeric_limits<double>::epsilon() < std::abs(normal.y) && std::abs(normal.y) < 1.0 + std::numeric_limits<double>::epsilon())
				y = 0.0;
			if (1.0 - std::numeric_limits<double>::epsilon() < std::abs(normal.z) && std::abs(normal.z) < 1.0 + std::numeric_limits<double>::epsilon())
				z = 0.0;
		}

		void check_range(model::Rules const& rules)
		{
			if (x < -rules.arena.width / 2.0 + rules.ROBOT_RADIUS)
				x = -rules.arena.width / 2.0 + rules.ROBOT_RADIUS;
			if (x > rules.arena.width / 2.0 - rules.ROBOT_RADIUS)
				x = rules.arena.width / 2.0 - rules.ROBOT_RADIUS;

			if (y < 0.0 + rules.ROBOT_RADIUS)
				y = 0.0 + rules.ROBOT_RADIUS;
			if (y > rules.arena.height - rules.ROBOT_RADIUS)
				y = rules.arena.height - rules.ROBOT_RADIUS;

			if (z < -rules.arena.depth / 2.0 - rules.arena.goal_depth + rules.ROBOT_RADIUS)
				z = -rules.arena.depth / 2.0 - rules.arena.goal_depth + rules.ROBOT_RADIUS;
			if (z > rules.arena.depth / 2.0 + rules.arena.goal_depth - rules.ROBOT_RADIUS)
				z = rules.arena.depth / 2.0 + rules.arena.goal_depth - rules.ROBOT_RADIUS;
		}
	};
}

MyStrategy::MyStrategy() = default;

void MyStrategy::act(model::Robot const& me, model::Rules const& rules, model::Game const& game, model::Action & action)
{
	sym::init(rules);

	sym::ball.position = { game.ball.x, game.ball.y, game.ball.z };
	sym::ball.velocity = { game.ball.velocity_x, game.ball.velocity_y, game.ball.velocity_z };

	for (int unused = 0; unused < 100; ++unused)
	{
		sym::tick();

	}

	static int forward = -1;
	static int goalkeeper = -1;

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

	if (me.id == forward)
	{
		point goal = { 0.0, 3.0 * rules.arena.goal_height / 4.0, rules.arena.depth / 2.0 };
		point ball = { game.ball.x, game.ball.y, game.ball.z };
		//ball = ball.add({ game.ball.velocity_x, game.ball.velocity_y, game.ball.velocity_z });
		auto normal = ball.sub(goal);
		normal.norm();
		normal.reverse();
		normal.mul(game.ball.radius + me.radius);
		auto hfp = ball.add(normal);
		hfp.check_range(rules);
		point fw = { me.x, me.y, me.z };
		//fw.add({ me.velocity_x, me.velocity_y, me.velocity_z });
		point speed = fw.sub(hfp);
		//speed.print("speed");
		if (speed.length() < rules.ROBOT_MAX_RADIUS)
			action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
		speed.drop_normal({ me.touch_normal_x, me.touch_normal_y, me.touch_normal_z });
		speed.norm();
		speed.mul(rules.ROBOT_MAX_GROUND_SPEED);
		action.target_velocity_x = speed.x;
		action.target_velocity_y = speed.y;
		action.target_velocity_z = speed.z;
	}

	if (me.id == goalkeeper)
	{
		point ball = { game.ball.x, game.ball.y, game.ball.z };
		point hfp = {
			[&]()
			{
				if (game.ball.x < -rules.arena.goal_width / 2.0 + rules.ROBOT_RADIUS * 2.0)
					return -rules.arena.goal_width / 2.0 + rules.ROBOT_RADIUS * 2.0;
				if (game.ball.x > rules.arena.goal_width / 2.0 - rules.ROBOT_RADIUS * 2.0)
					return rules.arena.goal_width / 2.0 - rules.ROBOT_RADIUS * 2.0;
				return game.ball.x;
			}(),
			rules.ROBOT_RADIUS, -rules.arena.depth / 2.0 - rules.arena.goal_side_radius - 0.3
		};
		point fw = { me.x, me.y, me.z };
		auto to_ball = fw.sub(ball);
		point speed = fw.sub(hfp);
		//speed.print("speed");
		if (to_ball.length() < rules.ROBOT_MAX_RADIUS + game.ball.radius)
			action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
		speed.drop_normal({ me.touch_normal_x, me.touch_normal_y, me.touch_normal_z });
		speed.norm();
		speed.mul(rules.ROBOT_MAX_GROUND_SPEED); // wtf?
		action.target_velocity_x = speed.x;
		action.target_velocity_y = speed.y;
		action.target_velocity_z = speed.z;
	}
}