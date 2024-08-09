const std = @import("std");
const shapes = @import("../shapes/shapes.zig");
const rl = @import("raylib");

pub const bitmask = std.bit_set.IntegerBitSet(64);

pub const ColliderType = union(enum) {
    Sphere: shapes.Sphere,
    AABB: shapes.AABB,
    OBB: shapes.AABB,

    pub fn createAABB(aabb: shapes.AABB) ColliderType {
        return .{ .AABB = aabb };
    }

    pub fn createOBB(aabb: shapes.AABB) ColliderType {
        return .{ .OBB = aabb };
    }

    pub fn createSphere(sphere: shapes.Sphere) ColliderType {
        return .{ .Sphere = sphere };
    }
};

const RayHitInfo = struct { info: rl.RayCollision, body: ?*RigidBody };
const CollisionInfo = struct {
    contact_points: [2]rl.Vector3,
    distance: f32,
    normal: rl.Vector3,

    pub fn init() CollisionInfo {
        return .{ .contact_points = .{ rl.Vector3.zero(), rl.Vector3.zero() }, .distance = 0.0, .normal = rl.Vector3.zero() };
    }
};

pub const Collider = struct {
    shape: ColliderType,
    mask: bitmask,
    layer: bitmask,

    pub fn init(shape: ColliderType) Collider {
        const empty = bitmask.initEmpty();
        return .{ .shape = shape, .mask = empty, .layer = empty };
    }

    fn getColliderIndex(collider: *const Collider, comptime index: u2) u8 {
        return switch (collider.shape) {
            ColliderType.AABB => 0b1 << (index * 2),
            ColliderType.Sphere => 0b10 << (index * 2),
            else => unreachable,
        };
    }

    pub fn intersectCollider(self: *const Collider, other: *const Collider, pos_1: rl.Vector3, pos_2: rl.Vector3) ?CollisionInfo {
        const a = getColliderIndex(self, 0);
        const b = getColliderIndex(other, 1);
        const c = a | b;

        return switch (c) {
            0b1010 => sphereVSphere(&self.shape.Sphere, &other.shape.Sphere, pos_1, pos_2),
            0b0101 => aabbVAABB(&self.shape.AABB, &other.shape.AABB, pos_1, pos_2),
            0b1001 => aabbVSphere(&self.shape.AABB, &other.shape.Sphere, pos_1, pos_2, false),
            0b0110 => aabbVSphere(&other.shape.AABB, &self.shape.Sphere, pos_2, pos_1, true),
            else => unreachable,
        };
    }

    fn sphereVSphere(a: *const shapes.Sphere, b: *const shapes.Sphere, pos_1: rl.Vector3, pos_2: rl.Vector3) ?CollisionInfo {
        const distance2 = pos_2.distanceSqr(pos_1);
        const radius_sum2: f32 = std.math.pow(f32, a.getRadius() + b.getRadius(), 2);

        var hit = CollisionInfo.init();

        if (distance2 > radius_sum2) {
            return null;
        }

        hit.distance = (a.getRadius() + b.getRadius()) - std.math.sqrt(distance2);
        hit.normal = pos_2.subtract(pos_1).normalize();
        hit.contact_points[0] = pos_1.add(hit.normal.scale(a.getRadius()));
        hit.contact_points[1] = pos_2.add(hit.normal.scale(a.getRadius()).negate());

        return hit;
    }

    fn aabbVAABB(a: *const shapes.AABB, b: *const shapes.AABB, pos_1: rl.Vector3, pos_2: rl.Vector3) ?CollisionInfo {
        const delta_pos = pos_2.subtract(pos_1);
        const half_extents_sum = a.half_extents.add(b.half_extents);

        var hit = CollisionInfo.init();

        const delta_x = @abs(delta_pos.x);
        const delta_y = @abs(delta_pos.y);
        const delta_z = @abs(delta_pos.z);

        if (delta_x >= half_extents_sum.x or delta_y >= half_extents_sum.y or delta_z >= half_extents_sum.z) {
            return null;
        }

        const normals = [_]rl.Vector3{
            rl.Vector3.init(-1, 0, 0), //left
            rl.Vector3.init(1, 0, 0), //right
            rl.Vector3.init(0, -1, 0), //bottom
            rl.Vector3.init(0, 1, 0), //top
            rl.Vector3.init(0, 0, -1), //forward
            rl.Vector3.init(0, 0, 1), //back
        };

        const min_a = a.getMin(pos_1);
        const max_a = a.getMax(pos_1);

        const min_b = b.getMin(pos_2);
        const max_b = b.getMax(pos_2);

        const distances = [_]f32{
            max_b.x - min_a.x,
            max_a.x - min_b.x,

            max_b.y - min_a.y,
            max_a.y - min_b.y,

            max_b.z - min_a.z,
            max_a.z - min_b.z,
        };

        hit.distance = std.math.inf(f32);

        for (0..6) |i| {
            if (distances[i] < hit.distance) {
                hit.distance = distances[i];
                hit.normal = normals[i];
            }
        }

        return hit;
    }

    fn aabbVSphere(a: *const shapes.AABB, b: *const shapes.Sphere, pos_1: rl.Vector3, pos_2: rl.Vector3, flipped: bool) ?CollisionInfo {
        const delta = pos_2.subtract(pos_1);
        const closest_point = delta.clamp(a.half_extents.negate(), a.half_extents);
        const local_point = delta.subtract(closest_point);

        const distance2 = local_point.lengthSqr();
        const radius2 = std.math.pow(f32, b.getRadius(), 2);

        var hit = CollisionInfo.init();

        if (distance2 > radius2) {
            return null;
        }

        hit.distance = local_point.length() - b.getRadius();
        hit.normal = if (flipped) local_point.normalize() else local_point.negate().normalize();

        hit.contact_points[0] = rl.Vector3.zero();
        hit.contact_points[1] = hit.normal.negate().scale(b.getRadius());

        return hit;
    }
};

pub const World = struct {
    colliders: std.ArrayList(*Collider),
    rigid_bodies: std.ArrayList(*RigidBody),

    gravity: rl.Vector3,

    pub fn init(allocator: std.mem.Allocator) World {
        // zig fmt: off
        return .{ 
            .colliders = std.ArrayList(*Collider).init(allocator), 
            .rigid_bodies = std.ArrayList(*RigidBody).init(allocator),
            .gravity = rl.Vector3.init(0, -9.8, 0),
        };
        // zig fmt: on
    }

    pub fn deinit(self: World) void {
        self.rigid_bodies.deinit();
        self.colliders.deinit();
    }

    pub fn addCollider(self: *World, collider: *Collider) !void {
        try self.colliders.append(collider);
    }

    pub fn addRigidBody(self: *World, rigid_body: *RigidBody) !void {
        try self.colliders.append(rigid_body.collider);
        try self.rigid_bodies.append(rigid_body);
    }

    fn getRayIntersectionCollider(ray: rl.Ray, body: *const RigidBody, mask: bitmask) rl.RayCollision {
        const set = mask.intersectWith(body.collider.layer);
        if (!set.eql(body.collider.layer) or set.eql(bitmask.initEmpty())) {
            return .{ .hit = false, .distance = std.math.inf(f32), .point = rl.Vector3.zero(), .normal = rl.Vector3.zero() };
        }

        return switch (body.collider.shape) {
            ColliderType.Sphere => |sphere| sphere.rayIntersectSphere(ray, body.position),
            ColliderType.AABB => |box| box.rayIntersectAABB(ray, body.position),
            ColliderType.OBB => |box| box.rayIntersectOBB(ray, body.orientation, body.position),
        };
    }

    pub fn getRayIntersection(self: *World, ray: rl.Ray, mask: bitmask) RayHitInfo {
        const hit_default = rl.RayCollision{ .hit = false, .distance = std.math.inf(f32), .point = rl.Vector3.zero(), .normal = rl.Vector3.zero() };
        var result: RayHitInfo = .{ .info = hit_default, .body = null };

        for (self.rigid_bodies.items) |body| {
            const new_hit = getRayIntersectionCollider(ray, body, mask);
            if (new_hit.distance < result.info.distance and new_hit.hit) {
                result.info = new_hit;
                result.body = body;
            }
        }

        return result;
    }

    pub fn integrate(self: *World, delta: f32) void {
        for (self.rigid_bodies.items) |body| {
            if (body.inverse_mass == 0) {
                continue;
            }

            if (body.use_gravity) {
                body.addForce(self.gravity.scale(body.mass));
            }

            var acceleration = body.force.scale(body.inverse_mass);

            body.linear_velocity = body.linear_velocity.add(acceleration.scale(delta));
            body.position = body.position.add(body.linear_velocity.scale(delta));

            body.damping_factor = std.math.clamp(body.damping_factor, 0, 1);
            const frame_damp = std.math.pow(f32, (1.0 - body.damping_factor), delta);
            body.linear_velocity = body.linear_velocity.scale(frame_damp);

            if (body.collider.shape == .AABB) {
                body.clearForce();
                continue;
            }

            const orientation = rl.Quaternion.fromMatrix(body.orientation).normalize().toMatrix();
            const inverse_orientation = orientation.transpose();

            body.inverse_inertia = inverse_orientation.multiply(body.inverse_inertia.multiply(body.orientation));

            const angular_acceleration = body.torque.transform(body.inverse_inertia);

            body.angular_velocity = body.angular_velocity.add(angular_acceleration.scale(delta));

            const angular_velocity_axis = body.angular_velocity.normalize();
            const angular_velocity_angle = body.angular_velocity.length() * delta;

            const new_orientation = rl.Quaternion.fromAxisAngle(angular_velocity_axis, angular_velocity_angle).normalize();

            body.orientation = body.orientation.multiply(new_orientation.toMatrix());

            body.angular_velocity = body.angular_velocity.scale(frame_damp);

            body.clearForce();
        }

        self.resolveCollisions();
    }

    fn resolveBodyCollisions(b1: *RigidBody, b2: *RigidBody) void {
        const info = b1.collider.intersectCollider(b2.collider, b1.position, b2.position);
        if (info) |hit| {
            b1.position = b1.position.subtract(hit.normal.scale(hit.distance));
        }
    }

    fn resolveCollisions(self: *World) void {
        for (self.rigid_bodies.items[0..], 0..) |b1, i| {
            for (self.rigid_bodies.items[0..], 0..) |b2, j| {
                if (i == j) continue;
                if (b1.mass == 0.0) continue;
                resolveBodyCollisions(b1, b2);
            }
        }
    }
};

pub const RigidBody = struct {
    collider: *Collider,

    inverse_mass: f32,
    mass: f32,

    linear_velocity: rl.Vector3,

    force: rl.Vector3,
    torque: rl.Vector3,

    position: rl.Vector3,
    damping_factor: f32,

    use_gravity: bool,

    inverse_inertia: rl.Matrix,
    angular_velocity: rl.Vector3,
    orientation: rl.Matrix,

    fn calculateInertiaTensor(mass: f32, collider_type: *ColliderType) rl.Matrix {
        var inertia_tensor = rl.Matrix.identity();

        switch (collider_type.*) {
            ColliderType.Sphere => |sphere| {
                const factor: f32 = 2.0 / 5.0;
                const inertia = 1.0 / (factor * mass * std.math.pow(f32, sphere.getRadius(), 2));
                inertia_tensor = rl.Matrix.scale(inertia, inertia, inertia);
            },
            ColliderType.AABB => |box| {
                const factor: f32 = 1.0 / 12.0;
                const size: rl.Vector3 = box.getSize();

                const size_x = std.math.pow(f32, size.x, 2);
                const size_y = std.math.pow(f32, size.y, 2);
                const size_z = std.math.pow(f32, size.z, 2);

                const inertia_x = factor * mass * (size_z + size_y);
                const inertia_y = factor * mass * (size_x + size_z);
                const inertia_z = factor * mass * (size_x + size_y);

                inertia_tensor = rl.Matrix.scale(1.0 / inertia_x, 1.0 / inertia_y, 1.0 / inertia_z);
            },
            ColliderType.OBB => |obb| {
                const factor: f32 = 1.0 / 12.0;
                const size: rl.Vector3 = obb.getSize();

                const size_x = std.math.pow(f32, size.x, 2);
                const size_y = std.math.pow(f32, size.y, 2);
                const size_z = std.math.pow(f32, size.z, 2);

                const inertia_x = factor * mass * (size_z + size_y);
                const inertia_y = factor * mass * (size_x + size_z);
                const inertia_z = factor * mass * (size_x + size_y);

                inertia_tensor = rl.Matrix.scale(1.0 / inertia_x, 1.0 / inertia_y, 1.0 / inertia_z);
            },
        }

        return inertia_tensor;
    }

    pub fn init(collider: *Collider, position: rl.Vector3) RigidBody {
        // zig fmt: off
        return .{ 
            .collider = collider, 
            .inverse_mass = 0, 
            .mass = 0,
            .linear_velocity = rl.Vector3.zero(), 
            .force = rl.Vector3.zero(),
            .position = position,
            .damping_factor = 0.05,
            .use_gravity = false,
            .inverse_inertia = rl.Matrix.identity(),
            .torque = rl.Vector3.zero(),
            .angular_velocity = rl.Vector3.zero(),
            .orientation = rl.Matrix.identity()
        };
        // zig fmt: on
    }

    pub fn setMass(self: *RigidBody, mass: f32) void {
        if (mass > 0) {
            self.inverse_mass = 1 / mass;
            self.mass = mass;
            self.inverse_inertia = calculateInertiaTensor(mass, &self.collider.shape);
        } else {
            std.debug.panic("Mass must be > 0!", .{});
        }
    }

    pub fn clearForce(self: *RigidBody) void {
        self.force = rl.Vector3.zero();
        self.torque = rl.Vector3.zero();
    }

    pub fn addForce(self: *RigidBody, force: rl.Vector3) void {
        self.force = self.force.add(force);
    }

    pub fn addForceAtPosition(self: *RigidBody, force: rl.Vector3, position: rl.Vector3) void {
        const local_pos = position.subtract(self.position);
        self.addForce(force);
        self.torque = self.torque.add(local_pos.crossProduct(force));
    }

    pub fn getLinearVelocity(self: *const RigidBody) rl.Vector3 {
        return self.linear_velocity;
    }
};

test "world leak" {
    const allocator = std.testing.allocator;
    var world = World.init(allocator);
    defer world.deinit();

    const box_shape = shapes.AABB.init(rl.Vector3.zero());
    var collider = Collider.init(ColliderType.createAABB(box_shape));

    try world.addCollider(&collider);
}

test "inverse tensor" {
    const allocator = std.testing.allocator;
    var world = World.init(allocator);
    defer world.deinit();

    const box_shape = shapes.AABB.init(rl.Vector3.init(0.5, 0.5, 0.5));
    var collider = Collider.init(ColliderType.createAABB(box_shape));
    var body = RigidBody.init(&collider, rl.Vector3.zero());
    body.setMass(1);

    const sphere_shape = shapes.Sphere.init(1.0);
    var sphere_collider = Collider.init(ColliderType.createSphere(sphere_shape));
    var ball_body = RigidBody.init(&sphere_collider, rl.Vector3.zero());
    ball_body.setMass(1);

    try world.addRigidBody(&body);
    try world.addRigidBody(&ball_body);

    const x: f32 = body.inverse_inertia.m0;
    const y: f32 = body.inverse_inertia.m5;
    const z: f32 = body.inverse_inertia.m10;

    const res: f32 = 6.0;

    try std.testing.expectEqual(res, x);
    try std.testing.expectEqual(res, y);
    try std.testing.expectEqual(res, z);

    const x2: f32 = ball_body.inverse_inertia.m0;
    const y2: f32 = ball_body.inverse_inertia.m5;
    const z2: f32 = ball_body.inverse_inertia.m10;

    const res2: f32 = 2.5;

    try std.testing.expectEqual(res2, x2);
    try std.testing.expectEqual(res2, y2);
    try std.testing.expectEqual(res2, z2);
}
