const std = @import("std");
const shapes = @import("../shapes/shapes.zig");
const rl = @import("raylib");

pub const bitmask = std.bit_set.IntegerBitSet(64);

pub const ColliderType = union(enum) {
    Sphere: *shapes.Sphere,
    AABB: *shapes.AABB,
    OBB: struct { aabb: *shapes.AABB, orientation: *rl.Matrix },

    pub fn createAABB(aabb: *shapes.AABB) ColliderType {
        return .{ .AABB = aabb };
    }

    pub fn createOBB(aabb: *shapes.AABB, orientation: *rl.Matrix) ColliderType {
        return .{ .OBB = .{ .aabb = aabb, .orientation = orientation } };
    }

    pub fn createSphere(sphere: *shapes.Sphere) ColliderType {
        return .{ .Sphere = sphere };
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
};

const RayHitInfo = struct { info: rl.RayCollision, body: ?*RigidBody };

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

    fn getRayIntersectionCollider(ray: rl.Ray, collider: *const Collider, collider_pos: rl.Vector3, mask: bitmask) rl.RayCollision {
        const set = mask.intersectWith(collider.layer);
        if (!set.eql(collider.layer) or set.eql(bitmask.initEmpty())) {
            return .{ .hit = false, .distance = std.math.inf(f32), .point = rl.Vector3.zero(), .normal = rl.Vector3.zero() };
        }

        return switch (collider.shape) {
            ColliderType.Sphere => |sphere| sphere.rayIntersectSphere(ray, collider_pos),
            ColliderType.AABB => |box| box.rayIntersectAABB(ray, collider_pos),
            ColliderType.OBB => |box| box.aabb.rayIntersectOBB(ray, box.orientation.*, collider_pos),
        };
    }

    pub fn getRayIntersection(self: *World, ray: rl.Ray, mask: bitmask) RayHitInfo {
        const hit_default = rl.RayCollision{ .hit = false, .distance = std.math.inf(f32), .point = rl.Vector3.zero(), .normal = rl.Vector3.zero() };
        var result: RayHitInfo = .{ .info = hit_default, .body = null };

        for (self.rigid_bodies.items) |body| {
            const new_hit = getRayIntersectionCollider(ray, body.collider, body.position, mask);
            if (new_hit.distance < hit_default.distance and new_hit.hit) {
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

            body.clearForce();
        }
    }
};

pub const RigidBody = struct {
    collider: *Collider,
    transform: rl.Matrix,

    inverse_mass: f32,
    mass: f32,

    linear_velocity: rl.Vector3,
    force: rl.Vector3,
    position: rl.Vector3,
    damping_factor: f32,

    use_gravity: bool,

    pub fn init(collider: *Collider, position: rl.Vector3) RigidBody {
        // zig fmt: off
        return .{ 
            .collider = collider, 
            .transform = rl.Matrix.identity(), 
            .inverse_mass = 0, 
            .mass = 0,
            .linear_velocity = rl.Vector3.zero(), 
            .force = rl.Vector3.zero(),
            .position = position,
            .damping_factor = 0.05,
            .use_gravity = false
        };
        // zig fmt: on
    }

    pub fn setMass(self: *RigidBody, mass: f32) void {
        if (mass > 0) {
            self.inverse_mass = 1 / mass;
            self.mass = mass;
        } else {
            std.debug.panic("Mass must be > 0!", .{});
        }
    }

    pub fn clearForce(self: *RigidBody) void {
        self.force = rl.Vector3.zero();
    }

    pub fn addForce(self: *RigidBody, force: rl.Vector3) void {
        self.force = self.force.add(force);
    }

    pub fn getLinearVelocity(self: *const RigidBody) rl.Vector3 {
        return self.linear_velocity;
    }
};

test "world leak" {
    const allocator = std.testing.allocator;
    var world = World.init(allocator);
    defer world.deinit();

    var box_shape = shapes.AABB.init(rl.Vector3.zero(), rl.Vector3.zero());
    var collider = Collider.init(ColliderType.createAABB(&box_shape));

    try world.addCollider(&collider);
}
