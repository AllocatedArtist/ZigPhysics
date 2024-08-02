const std = @import("std");
const shapes = @import("../shapes/shapes.zig");
const rl = @import("raylib");

pub const bitmask = std.bit_set.IntegerBitSet(64);

pub const ColliderType = union(enum) {
    Sphere: *shapes.Sphere,
    AABB: *shapes.AABB,
    OBB: struct { aabb: *shapes.AABB, orientation: *rl.Matrix },
    Triangle: *shapes.Triangle,
    Plane: *shapes.Plane,

    pub fn createAABB(aabb: *shapes.AABB) ColliderType {
        return .{ .AABB = aabb };
    }

    pub fn createOBB(aabb: *shapes.AABB, orientation: *rl.Matrix) ColliderType {
        return .{ .OBB = .{ .aabb = aabb, .orientation = orientation } };
    }

    pub fn createSphere(sphere: *shapes.Sphere) ColliderType {
        return .{ .Sphere = sphere };
    }

    pub fn createTriangle(triangle: *shapes.Triangle) ColliderType {
        return .{ .Triangle = triangle };
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

pub const World = struct {
    colliders: std.ArrayList(*Collider),

    pub fn init(allocator: std.mem.Allocator) World {
        return .{ .colliders = std.ArrayList(*Collider).init(allocator) };
    }

    pub fn deinit(self: World) void {
        self.colliders.deinit();
    }

    pub fn addCollider(self: *World, collider: *Collider) !void {
        try self.colliders.append(collider);
    }

    pub fn getRayIntersection(self: *World, ray: rl.Ray, mask: bitmask) rl.RayCollision {
        var hit = rl.RayCollision{ .hit = false, .distance = std.math.inf(f32), .point = rl.Vector3.zero(), .normal = rl.Vector3.zero() };
        for (self.colliders.items) |collider| {
            const set = mask.intersectWith(collider.layer);
            if (!set.eql(collider.layer) or set.eql(bitmask.initEmpty())) {
                continue;
            }
            switch (collider.shape) {
                ColliderType.Sphere => |sphere| {
                    const new_hit = sphere.rayIntersectSphere(ray);
                    if (new_hit.hit and new_hit.distance < hit.distance) {
                        hit = new_hit;
                    }
                },
                ColliderType.AABB => |box| {
                    const new_hit = box.rayIntersectAABB(ray);
                    if (new_hit.hit and new_hit.distance < hit.distance) {
                        hit = new_hit;
                    }
                },
                ColliderType.OBB => |box| {
                    const new_hit = box.aabb.rayIntersectOBB(box.orientation.*, ray);
                    if (new_hit.hit and new_hit.distance < hit.distance) {
                        hit = new_hit;
                    }
                },
                ColliderType.Triangle => |triangle| {
                    const new_hit = triangle.rayIntersectTriangleBT(ray);
                    if (new_hit.hit and new_hit.distance < hit.distance) {
                        hit = new_hit;
                    }
                },
                else => {},
            }
        }
        return hit;
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
