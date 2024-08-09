const std = @import("std");
const rl = @import("raylib");
const shape = @import("shapes/shapes.zig");
const physics = @import("physics/physics.zig");

// zig fmt: off
var camera = rl.Camera3D {
  .position = rl.Vector3.init(0, 1, 3),
  .target = rl.Vector3.init(0, 0, -1),
  .up = rl.Vector3.init(0, 1, 0),
  .fovy = 90,
  .projection = .camera_perspective
};
// zig fmt: on

var yaw: f32 = 0;
var pitch: f32 = 0;
var forward = rl.Vector3.init(0, 0, -1);

fn updateCamera(cam: *rl.Camera) void {
    const right = forward.crossProduct(cam.up);

    var dir = rl.Vector3.zero();
    if (rl.isKeyDown(.key_w)) {
        dir = dir.add(forward);
    }

    if (rl.isKeyDown(.key_s)) {
        dir = dir.add(forward.negate());
    }

    if (rl.isKeyDown(.key_a)) {
        dir = dir.add(right.negate());
    }

    if (rl.isKeyDown(.key_d)) {
        dir = dir.add(right);
    }

    dir = dir.normalize();
    const speed = 5;
    const delta = rl.getFrameTime();

    cam.position = cam.position.add(dir.scale(speed * delta));
    cam.target = cam.position.add(forward);

    if (!rl.isMouseButtonDown(.mouse_button_right)) {
        return;
    }

    const sensitivity = 0.1;
    yaw -= rl.getMouseDelta().x * sensitivity;
    yaw = @mod(yaw, 360);
    pitch -= rl.getMouseDelta().y * sensitivity;
    pitch = std.math.clamp(pitch, -89, 89);

    forward = rl.Vector3.init(0, 0, -1);

    const yaw_rad = std.math.degreesToRadians(yaw);
    const pitch_rad = std.math.degreesToRadians(pitch);

    const rot = rl.Quaternion.fromEuler(pitch_rad, yaw_rad, 0);
    forward = forward.rotateByQuaternion(rot);
}

const GameObject = struct {
    body: physics.RigidBody,
    collider: physics.Collider,

    fn initSphere(allocator: std.mem.Allocator, radius: f32, pos: rl.Vector3) !*GameObject {
        var game_object = try allocator.create(GameObject);
        errdefer allocator.destroy(game_object);

        const sphere = shape.Sphere.init(radius);
        game_object.collider = physics.Collider.init(physics.ColliderType.createSphere(sphere));
        game_object.collider.layer.setValue(0, true);
        game_object.body = physics.RigidBody.init(&game_object.collider, pos);
        game_object.body.setMass(1.0);
        game_object.body.use_gravity = true;
        game_object.body.damping_factor = 0.65;

        return game_object;
    }
};

test "free" {
    const allocator = std.testing.allocator;
    var game_objects = std.ArrayList(*GameObject).init(allocator);
    defer {
        for (game_objects.items) |obj| {
            allocator.destroy(obj);
        }
        game_objects.deinit();
    }

    try game_objects.append(try GameObject.initSphere(allocator, 1.0, rl.Vector3.zero()));
}

pub fn main() !void {
    rl.initWindow(1600, 1480, "Physics");

    rl.setTargetFPS(60);

    camera.target = camera.position.add(rl.Vector3.init(0, 0, -1));

    const allocator = std.heap.page_allocator;

    const logo = rl.loadTexture("assets/gitlogo.png");
    defer rl.unloadTexture(logo);

    var smiley_ball = rl.loadModelFromMesh(rl.genMeshSphere(1.0, 10.0, 10.0));
    defer rl.unloadModel(smiley_ball);

    rl.setMaterialTexture(@ptrCast(&smiley_ball.materials[0]), .material_map_albedo, logo);

    var world = physics.World.init(allocator);
    defer world.deinit();

    // var game_objects = std.ArrayList(*GameObject).init(allocator);
    // defer {
    //     for (game_objects.items) |obj| {
    //         allocator.destroy(obj);
    //     }
    //     game_objects.deinit();
    // }

    var small_box = physics.Collider.init(physics.ColliderType.createAABB(shape.AABB.init(rl.Vector3.init(0.5, 0.5, 0.5))));
    var big_box = physics.Collider.init(physics.ColliderType.createAABB(shape.AABB.init(rl.Vector3.init(2.0, 1.0, 1.0))));
    var ball_shape = physics.Collider.init(physics.ColliderType.createSphere(shape.Sphere.init(2.0)));
    var small_ball = physics.Collider.init(physics.ColliderType.createSphere(shape.Sphere.init(0.5)));
    var ground_shape = physics.Collider.init(physics.ColliderType.createAABB(shape.AABB.init(rl.Vector3.init(25.0, 0.1, 25.0))));

    var player_body = physics.RigidBody.init(&small_box, rl.Vector3.init(0, 0, 5));
    var static_body = physics.RigidBody.init(&big_box, rl.Vector3.zero());
    var ball_body = physics.RigidBody.init(&ball_shape, rl.Vector3.init(1.0, 0.0, 0.0));
    var small_ball_body = physics.RigidBody.init(&small_ball, rl.Vector3.init(0, 0, -3));
    var ground_body = physics.RigidBody.init(&ground_shape, rl.Vector3.init(0, -0.8, 0));

    ground_body.use_gravity = false;

    small_ball.layer.setValue(0, true);

    small_ball_body.setMass(1.0);
    small_ball_body.use_gravity = true;
    small_ball_body.damping_factor = 0.7;

    ball_body.use_gravity = false;
    ball_body.damping_factor = 0.7;

    try world.addRigidBody(&player_body);
    try world.addRigidBody(&static_body);
    try world.addRigidBody(&ball_body);
    try world.addRigidBody(&small_ball_body);
    try world.addRigidBody(&ground_body);

    player_body.setMass(1.0);
    player_body.use_gravity = true;
    player_body.damping_factor = 0.7;

    small_box.layer.setValue(0, true);

    static_body.use_gravity = false;
    static_body.damping_factor = 0.7;

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        rl.clearBackground(rl.Color.black);
        updateCamera(&camera);

        world.integrate(rl.getFrameTime());

        const pick = rl.getScreenToWorldRay(rl.getMousePosition(), camera);
        const ball_intersection = world.getRayIntersection(pick, physics.bitmask.initFull());

        if (rl.isMouseButtonDown(.mouse_button_left)) {
            if (ball_intersection.body) |b| {
                b.addForce(ball_intersection.info.normal.negate().scale(100));
            }
        }

        if (rl.isMouseButtonDown(.mouse_button_right)) {
            if (ball_intersection.body) |b| {
                b.addForce(rl.Vector3.init(0, 100, 0));
            }
        }

        camera.begin();

        rl.drawGrid(10, 1.0);

        rl.drawCubeWiresV(player_body.position, player_body.collider.shape.AABB.getSize(), rl.Color.maroon);
        rl.drawCubeWiresV(static_body.position, static_body.collider.shape.AABB.getSize(), rl.Color.green);
        rl.drawCubeV(ground_body.position, ground_body.collider.shape.AABB.getSize(), rl.Color.dark_brown);

        rl.drawSphere(ball_body.position, ball_shape.shape.Sphere.getRadius(), rl.Color.yellow);
        rl.drawSphere(small_ball_body.position, small_ball.shape.Sphere.getRadius(), rl.Color.pink);

        camera.end();

        rl.endDrawing();
    }
}
