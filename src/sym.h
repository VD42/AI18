#include "model/Rules.h"

#include <cmath>
#include <optional>
#include <vector>
#include <algorithm>

namespace sym
{
	static bool inited = false;
	static double MIN_HIT_E;
	static double MAX_HIT_E;
	static double GRAVITY;
	static double MAX_ENTITY_SPEED;
	static double ROBOT_MAX_GROUND_SPEED;
	static double ROBOT_ACCELERATION;
	static double NITRO_POINT_VELOCITY_CHANGE;
	static double ROBOT_NITRO_ACCELERATION;
	static double ROBOT_MIN_RADIUS;
	static double ROBOT_MAX_RADIUS;
	static double ROBOT_MAX_JUMP_SPEED;
	static double MAX_NITRO_AMOUNT;
	static int NITRO_PACK_RESPAWN_TICKS;
	static int TICKS_PER_SECOND;
	static int MICROTICKS_PER_TICK;

	struct Vec2D
	{
		double x;
		double y;
	};

	Vec2D operator+(Vec2D const& a, Vec2D const& b)
	{
		return { a.x + b.x, a.y + b.y };
	}

	Vec2D operator-(Vec2D const& a, Vec2D const& b)
	{
		return { a.x - b.x, a.y - b.y };
	}

	double length(Vec2D const& a)
	{
		return std::sqrt(a.x * a.x + a.y * a.y);
	}

	Vec2D normalize(Vec2D const& a)
	{
		auto l = length(a);
		return { a.x / l, a.y / l };
	}

	Vec2D operator*(Vec2D const& a, double b)
	{
		return { a.x * b, a.y * b };
	}

	Vec2D operator/(Vec2D const& a, double b)
	{
		return { a.x / b, a.y / b };
	}


	struct Vec3D
	{
		double x;
		double y;
		double z;

		void operator+=(Vec3D const& a)
		{
			x += a.x;
			y += a.y;
			z += a.z;
		}

		void operator-=(Vec3D const& a)
		{
			x -= a.x;
			y -= a.y;
			z -= a.z;
		}
	};

	Vec3D operator-(Vec3D const& a, Vec3D const& b)
	{
		return { a.x - b.x, a.y - b.y, a.z - b.z };
	}

	double dot(Vec3D const& a, Vec3D const& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	double length(Vec3D const& a)
	{
		return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
	}

	Vec3D normalize(Vec3D const& a)
	{
		auto l = length(a);
		return { a.x / l, a.y / l, a.z / l };
	}

	Vec3D operator*(Vec3D const& a, double b)
	{
		return { a.x * b, a.y * b, a.z * b };
	}

	struct disnorm
	{
		double distance;
		Vec3D normal;
	};

	disnorm min(disnorm const& a, disnorm const& b)
	{
		if (b.distance < a.distance)
			return b;
		return a;
	}

	double clamp(double value, double min, double max)
	{
		if (value < min)
			return min;
		if (value > max)
			return max;
		return value;
	}

	Vec3D clamp(Vec3D const& value, double max)
	{
		auto l = length(value);
		if (l < max)
			return value;
		auto n = normalize(value);
		return n * max;
	}

	struct Entity
	{
		Vec3D position;
		double mass;
		double radius;
		Vec3D velocity;
		double radius_change_speed;
		double arena_e;
	};

	double random(double min, double max)
	{
		return (max - min) / 2.0;
	}

	disnorm dan_to_plane(Vec3D const& point, Vec3D const& point_on_plane, Vec3D const& plane_normal)
	{
		return {
			dot(point - point_on_plane, plane_normal),
			plane_normal
		};
	}

	disnorm dan_to_sphere_inner(Vec3D const& point, Vec3D const& sphere_center, double sphere_radius)
	{
		return {
			sphere_radius - length(point - sphere_center),
			normalize(sphere_center - point)
		};
	}

	disnorm dan_to_sphere_outer(Vec3D const& point, Vec3D const& sphere_center, double sphere_radius)
	{
		return {
			length(point - sphere_center) - sphere_radius,
			normalize(point - sphere_center)
		};
	}

	struct Arena
	{
		double width;
		double height;
		double depth;
		double goal_width;
		double goal_height;
		double goal_depth;
		double goal_top_radius;
		double goal_side_radius;
		double bottom_radius;
		double corner_radius;
		double top_radius;
	};

	static Arena arena;

	disnorm dan_to_arena_quarter(Vec3D const& point)
	{
		// Ground
		auto dan = dan_to_plane(point, { 0.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 });

		// Ceiling
		dan = min(dan, dan_to_plane(point, { 0.0, arena.height, 0.0 }, { 0.0, -1.0, 0.0 }));

		// Side x
		dan = min(dan, dan_to_plane(point, { arena.width / 2.0, 0.0, 0.0 }, { -1.0, 0.0, 0.0 }));

		// Side z (goal)
		dan = min(dan, dan_to_plane(
				point,
				{ 0.0, 0.0, (arena.depth / 2.0) + arena.goal_depth },
				{ 0.0, 0.0, -1.0 }));

		// Side z
		auto v = Vec2D { point.x, point.y } - Vec2D {
				(arena.goal_width / 2.0) - arena.goal_top_radius,
				arena.goal_height - arena.goal_top_radius };
		if (point.x >= (arena.goal_width / 2.0) + arena.goal_side_radius
			|| point.y >= arena.goal_height + arena.goal_side_radius
			|| (
				v.x > 0.0
				&& v.y > 0.0
				&& length(v) >= arena.goal_top_radius + arena.goal_side_radius))
		{
			dan = min(dan, dan_to_plane(point, { 0.0, 0.0, arena.depth / 2.0 }, { 0.0, 0.0, -1.0 }));
		}

		// Side x & ceiling (goal)
		if (point.z >= (arena.depth / 2.0) + arena.goal_side_radius)
		{
			// x
			dan = min(dan, dan_to_plane(
					point,
					{ arena.goal_width / 2.0, 0.0, 0.0 },
					{ -1.0, 0.0, 0.0 }));
			// y
			dan = min(dan, dan_to_plane(point, { 0.0, arena.goal_height, 0.0 }, { 0.0, -1.0, 0.0 }));
		}

		// Goal back corners
		//assert arena.bottom_radius == arena.goal_top_radius
		if (point.z > (arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius)
		{
			dan = min(dan, dan_to_sphere_inner(
					point,
					{
						clamp(
							point.x,
							arena.bottom_radius - (arena.goal_width / 2.0),
							(arena.goal_width / 2.0) - arena.bottom_radius
						),
						clamp(
							point.y,
							arena.bottom_radius,
							arena.goal_height - arena.goal_top_radius
						),
						(arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius},
					arena.bottom_radius));
		}

		// Corner
		if (point.x > (arena.width / 2.0) - arena.corner_radius
				&& point.z > (arena.depth / 2.0) - arena.corner_radius)
		{
			dan = min(dan, dan_to_sphere_inner(
					point,
					{
						(arena.width / 2.0) - arena.corner_radius,
						point.y,
						(arena.depth / 2.0) - arena.corner_radius
					},
					arena.corner_radius));
		}

		// Goal outer corner
		if (point.z < (arena.depth / 2.0) + arena.goal_side_radius)
		{
			// Side x
			if (point.x < (arena.goal_width / 2.0) + arena.goal_side_radius)
			{
				dan = min(dan, dan_to_sphere_outer(
						point,
						{
							(arena.goal_width / 2.0) + arena.goal_side_radius,
							point.y,
							(arena.depth / 2.0) + arena.goal_side_radius
						},
						arena.goal_side_radius));
			}
			// Ceiling
			if (point.y < arena.goal_height + arena.goal_side_radius)
			{
				dan = min(dan, dan_to_sphere_outer(
						point,
						{
							point.x,
							arena.goal_height + arena.goal_side_radius,
							(arena.depth / 2.0) + arena.goal_side_radius
						},
						arena.goal_side_radius));
			}
			// Top corner
			auto o = Vec2D {
				(arena.goal_width / 2.0) - arena.goal_top_radius,
				arena.goal_height - arena.goal_top_radius
			};
			auto v = Vec2D { point.x, point.y } - o;
			if (v.x > 0.0 && v.y > 0.0)
			{
				o = o + normalize(v) * (arena.goal_top_radius + arena.goal_side_radius);
				dan = min(dan, dan_to_sphere_outer(
						point,
						{ o.x, o.y, (arena.depth / 2.0) + arena.goal_side_radius },
						arena.goal_side_radius));
			}
		}

		// Goal inside top corners
		if (point.z > (arena.depth / 2.0) + arena.goal_side_radius
				&& point.y > arena.goal_height - arena.goal_top_radius)
		{
			// Side x
			if (point.x > (arena.goal_width / 2.0) - arena.goal_top_radius)
			{
				dan = min(dan, dan_to_sphere_inner(
						point,
						{
							(arena.goal_width / 2.0) - arena.goal_top_radius,
							arena.goal_height - arena.goal_top_radius,
							point.z
						},
						arena.goal_top_radius));
			}
			// Side z
			if (point.z > (arena.depth / 2.0) + arena.goal_depth - arena.goal_top_radius)
			{
				dan = min(dan, dan_to_sphere_inner(
						point,
						{
							point.x,
							arena.goal_height - arena.goal_top_radius,
							(arena.depth / 2.0) + arena.goal_depth - arena.goal_top_radius
						},
						arena.goal_top_radius));
			}
		}

		// Bottom corners
		if (point.y < arena.bottom_radius)
		{
			// Side x
			if (point.x > (arena.width / 2.0) - arena.bottom_radius)
			{
				dan = min(dan, dan_to_sphere_inner(
						point,
						{
							(arena.width / 2.0) - arena.bottom_radius,
							arena.bottom_radius,
							point.z
						},
						arena.bottom_radius));
			}
			// Side z
			if (point.z > (arena.depth / 2.0) - arena.bottom_radius
					&& point.x >= (arena.goal_width / 2.0) + arena.goal_side_radius)
			{
				dan = min(dan, dan_to_sphere_inner(
						point,
						{
							point.x,
							arena.bottom_radius,
							(arena.depth / 2.0) - arena.bottom_radius
						},
						arena.bottom_radius));
			}
			// Side z (goal)
			if (point.z > (arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius)
			{
				dan = min(dan, dan_to_sphere_inner(
						point,
						{
							point.x,
							arena.bottom_radius,
							(arena.depth / 2.0) + arena.goal_depth - arena.bottom_radius
						},
						arena.bottom_radius));
			}
			// Goal outer corner
			auto o = Vec2D {
				(arena.goal_width / 2.0) + arena.goal_side_radius,
				(arena.depth / 2.0) + arena.goal_side_radius
			};
			auto v = Vec2D { point.x, point.z } - o;
			if (v.x < 0.0 && v.y < 0.0
					&& length(v) < arena.goal_side_radius + arena.bottom_radius)
			{
				o = o + normalize(v) * (arena.goal_side_radius + arena.bottom_radius);
				dan = min(dan, dan_to_sphere_inner(
						point,
						{ o.x, arena.bottom_radius, o.y },
						arena.bottom_radius));
			}
			// Side x (goal)
			if (point.z >= (arena.depth / 2.0) + arena.goal_side_radius
					&& point.x > (arena.goal_width / 2.0) - arena.bottom_radius)
			{
				dan = min(dan, dan_to_sphere_inner(
						point,
						{
							(arena.goal_width / 2.0) - arena.bottom_radius,
							arena.bottom_radius,
							point.z
						},
						arena.bottom_radius));
			}
			// Corner
			if (point.x > (arena.width / 2.0) - arena.corner_radius
					&& point.z > (arena.depth / 2.0) - arena.corner_radius)
			{
				auto corner_o = Vec2D {
					(arena.width / 2.0) - arena.corner_radius,
					(arena.depth / 2.0) - arena.corner_radius
				};
				auto n = Vec2D { point.x, point.z } - corner_o;
				auto dist = length(n);
				if (dist > arena.corner_radius - arena.bottom_radius)
				{
					n = n / dist;
					auto o2 = corner_o + n * (arena.corner_radius - arena.bottom_radius);
					dan = min(dan, dan_to_sphere_inner(
							point,
							{ o2.x, arena.bottom_radius, o2.y },
							arena.bottom_radius));
				}
			}
		}

		// Ceiling corners
		if (point.y > arena.height - arena.top_radius)
		{
			// Side x
			if (point.x > (arena.width / 2.0) - arena.top_radius)
			{
				dan = min(dan, dan_to_sphere_inner(
						point,
						{
							(arena.width / 2.0) - arena.top_radius,
							arena.height - arena.top_radius,
							point.z,
						},
						arena.top_radius));
			}
			// Side z
			if (point.z > (arena.depth / 2.0) - arena.top_radius)
			{
				dan = min(dan, dan_to_sphere_inner(
						point,
						{
							point.x,
							arena.height - arena.top_radius,
							(arena.depth / 2.0) - arena.top_radius
						},
						arena.top_radius));
			}

			// Corner
			if (point.x > (arena.width / 2.0) - arena.corner_radius
					&& point.z > (arena.depth / 2.0) - arena.corner_radius)
			{
				auto corner_o = Vec2D {
					(arena.width / 2.0) - arena.corner_radius,
					(arena.depth / 2.0) - arena.corner_radius
				};
				auto dv = Vec2D { point.x, point.z } - corner_o;
				if (length(dv) > arena.corner_radius - arena.top_radius)
				{
					auto n = normalize(dv);
					auto o2 = corner_o + n * (arena.corner_radius - arena.top_radius);
					dan = min(dan, dan_to_sphere_inner(
							point,
							{ o2.x, arena.height - arena.top_radius, o2.y },
							arena.top_radius));
				}
			}
		}
    
		return dan;
	}

	disnorm dan_to_arena(Vec3D point)
	{
		auto negate_x = point.x < 0.0;
		auto negate_z = point.z < 0.0;
		if (negate_x)
			point.x = -point.x;
		if (negate_z)
			point.z = -point.z;
		auto result = dan_to_arena_quarter(point);
		if (negate_x)
			result.normal.x = -result.normal.x;
		if (negate_z)
			result.normal.z = -result.normal.z;
		return result;
	}


	void collide_entities(Entity & a, Entity & b)
	{
		auto delta_position = b.position - a.position;
		auto distance = length(delta_position);
		auto penetration = a.radius + b.radius - distance;
		if (penetration > 0.0)
		{
			auto k_a = (1.0 / a.mass) / ((1.0 / a.mass) + (1.0 / b.mass));
			auto k_b = (1.0 / b.mass) / ((1.0 / a.mass) + (1.0 / b.mass));
			auto normal = normalize(delta_position);
			a.position -= normal * penetration * k_a;
			b.position += normal * penetration * k_b;
			auto delta_velocity = dot(b.velocity - a.velocity, normal)
				+ b.radius_change_speed - a.radius_change_speed;
			if (delta_velocity < 0.0)
			{
				auto impulse = normal * ((1.0 + random(MIN_HIT_E, MAX_HIT_E)) * delta_velocity);
				a.velocity += impulse * k_a;
				b.velocity -= impulse * k_b;
			}
		}
	}

	std::optional<Vec3D> collide_with_arena(Entity & e)
	{
		auto [distance, normal] = dan_to_arena(e.position);
		auto penetration = e.radius - distance;
		if (penetration > 0.0)
		{
			e.position += normal * penetration;
			auto velocity = dot(e.velocity, normal) - e.radius_change_speed;
			if (velocity < 0.0)
			{
				e.velocity -= normal * ((1.0 + e.arena_e) * velocity);
				return normal;
			}
		}
		return std::nullopt;
	}

	void move(Entity & e, double delta_time)
	{
		e.velocity = clamp(e.velocity, MAX_ENTITY_SPEED);
		e.position += e.velocity * delta_time;
		e.position.y -= GRAVITY * delta_time * delta_time / 2.0;
		e.velocity.y -= GRAVITY * delta_time;
	}

	struct Robot : Entity
	{
		bool touch;
		Vec3D touch_normal;

		double nitro;

		struct
		{
			Vec3D target_velocity;
			bool use_nitro;
			double jump_speed;
		} action;
	};

	void shuffle(std::vector<Robot> & robots)
	{
		// nothing
	}

	static std::vector<Robot> robots;

	double max(double a, double b)
	{
		return std::max(a, b);
	}

	struct Ball : Entity
	{
	};

	static Ball ball;

	void goal_scored()
	{
		// nothing
	}

	struct Pack : Entity
	{
		bool alive;
		int respawn_ticks;
	};

	static std::vector<Pack> nitro_packs;

	void update(double delta_time)
	{
		shuffle(robots);

		for (auto & robot : robots)
		{
			if (robot.touch)
			{
				auto target_velocity = clamp(
					robot.action.target_velocity,
					ROBOT_MAX_GROUND_SPEED);
				target_velocity -= robot.touch_normal
					* dot(robot.touch_normal, target_velocity);
				auto target_velocity_change = target_velocity - robot.velocity;
				if (length(target_velocity_change) > 0.0)
				{
					auto acceleration = ROBOT_ACCELERATION * max(0.0, robot.touch_normal.y);
					robot.velocity += clamp(
						normalize(target_velocity_change) * (acceleration * delta_time),
						length(target_velocity_change));
				}
			}

			if (robot.action.use_nitro)
			{
				auto target_velocity_change = clamp(
					robot.action.target_velocity - robot.velocity,
					robot.nitro * NITRO_POINT_VELOCITY_CHANGE);
				if (length(target_velocity_change) > 0.0)
				{
					auto acceleration = normalize(target_velocity_change)
						* ROBOT_NITRO_ACCELERATION;
					auto velocity_change = clamp(
						acceleration * delta_time,
						length(target_velocity_change));
					robot.velocity += velocity_change;
					robot.nitro -= length(velocity_change)
						/ NITRO_POINT_VELOCITY_CHANGE;
				}
			}

			move(robot, delta_time);
			robot.radius = ROBOT_MIN_RADIUS + (ROBOT_MAX_RADIUS - ROBOT_MIN_RADIUS)
						* robot.action.jump_speed / ROBOT_MAX_JUMP_SPEED;
			robot.radius_change_speed = robot.action.jump_speed;
		}

		move(ball, delta_time);

		for (int i = 0; i < (int)robots.size(); ++i)
			for (int j = 0; j < i; ++j)
				collide_entities(robots[i], robots[j]);

		for (auto & robot : robots)
		{
			collide_entities(robot, ball);
			auto collision_normal = collide_with_arena(robot);
			if (!collision_normal.has_value())
			{
				robot.touch = false;
			}
			else
			{
				robot.touch = true;
				robot.touch_normal = collision_normal.value();
			}
		}
		collide_with_arena(ball);

		if (abs(ball.position.z) > arena.depth / 2.0 + ball.radius)
			goal_scored();

		for (auto & robot : robots)
		{
			if (robot.nitro == MAX_NITRO_AMOUNT)
				continue;
			for (auto & pack : nitro_packs)
			{
				if (!pack.alive)
					continue;
				if (length(robot.position - pack.position) <= robot.radius + pack.radius)
				{
					robot.nitro = MAX_NITRO_AMOUNT;
					pack.alive = false;
					pack.respawn_ticks = NITRO_PACK_RESPAWN_TICKS;
				}
			}
		}
	}

	void tick()
	{
		auto delta_time = 1.0 / (double)TICKS_PER_SECOND;
		for (int unused = 0; unused < MICROTICKS_PER_TICK; ++unused)
			update(delta_time / (double)MICROTICKS_PER_TICK);

		for (auto & pack : nitro_packs)
		{
			if (pack.alive)
				continue;
			pack.respawn_ticks -= 1;
			if (pack.respawn_ticks == 0)
				pack.alive = true;
		}
	}

	void init(model::Rules const& rules)
	{
		if (inited)
			return;

		MIN_HIT_E = rules.MAX_HIT_E;
		MAX_HIT_E = rules.MAX_HIT_E;
		GRAVITY = rules.GRAVITY;
		MAX_ENTITY_SPEED = rules.MAX_ENTITY_SPEED;
		ROBOT_MAX_GROUND_SPEED = rules.ROBOT_MAX_GROUND_SPEED;
		ROBOT_ACCELERATION = rules.ROBOT_ACCELERATION;
		NITRO_POINT_VELOCITY_CHANGE = rules.NITRO_POINT_VELOCITY_CHANGE;
		ROBOT_NITRO_ACCELERATION = rules.ROBOT_NITRO_ACCELERATION;
		ROBOT_MIN_RADIUS = rules.ROBOT_MIN_RADIUS;
		ROBOT_MAX_RADIUS = rules.ROBOT_MAX_RADIUS;
		ROBOT_MAX_JUMP_SPEED = rules.ROBOT_MAX_JUMP_SPEED;
		MAX_NITRO_AMOUNT = rules.MAX_NITRO_AMOUNT;
		NITRO_PACK_RESPAWN_TICKS = rules.NITRO_PACK_RESPAWN_TICKS;
		TICKS_PER_SECOND = rules.TICKS_PER_SECOND;
		MICROTICKS_PER_TICK = rules.MICROTICKS_PER_TICK;

		arena.width = rules.arena.width;
		arena.height = rules.arena.height;
		arena.depth = rules.arena.depth;
		arena.goal_width = rules.arena.goal_width;
		arena.goal_height = rules.arena.goal_height;
		arena.goal_depth = rules.arena.goal_depth;
		arena.goal_top_radius = rules.arena.goal_top_radius;
		arena.goal_side_radius = rules.arena.goal_side_radius;
		arena.bottom_radius = rules.arena.bottom_radius;
		arena.corner_radius = rules.arena.corner_radius;
		arena.top_radius = rules.arena.top_radius;

		ball.arena_e = rules.BALL_ARENA_E;
		ball.mass = rules.BALL_MASS;
		ball.radius = rules.BALL_RADIUS;
		ball.radius_change_speed = 0.0;

		inited = true;
	}
}