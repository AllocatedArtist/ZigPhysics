const rl = @import("raylib");
const std = @import("std");

pub const Triangle = struct {
    points: [3]rl.Vector3,
    distance: f32,

    //ccw ordering
    pub fn init(p1: rl.Vector3, p2: rl.Vector3, p3: rl.Vector3) Triangle {
        const points = [3]rl.Vector3{ p1, p2, p3 };
        const normal = getNormalFromPoints(points);
        const distance = normal.dotProduct(p1);

        return .{ .points = points, .distance = distance };
    }

    pub fn getDistance(self: *const Triangle) f32 {
        return self.distance;
    }

    pub fn getPoints(self: *const Triangle) [3]rl.Vector3 {
        return self.points;
    }

    fn getNormalFromPoints(points: [3]rl.Vector3) rl.Vector3 {
        const point_a = points[0];
        const side_ba = points[1].subtract(point_a);
        const side_ca = points[2].subtract(point_a);
        return side_ba.crossProduct(side_ca).normalize();
    }

    pub fn getNormal(self: *const Triangle) rl.Vector3 {
        return getNormalFromPoints(self.points);
    }

    pub fn getCenter(self: *const Triangle) rl.Vector3 {
        const p1 = self.points[0];
        const p2 = self.points[1];
        const p3 = self.points[2];

        const center_x = (p1.x + p2.x + p3.x) / 3;
        const center_y = (p1.y + p2.y + p3.y) / 3;
        const center_z = (p1.z + p2.z + p3.z) / 3;

        return rl.Vector3.init(center_x, center_y, center_z);
    }

    pub fn rayIntersectTriangle(self: *const Triangle, ray: rl.Ray) rl.RayCollision {
        const zero = rl.Vector3.zero();
        var hit = rl.RayCollision{ .hit = false, .point = zero, .normal = zero, .distance = 0 };

        const e1 = self.points[1].subtract(self.points[0]);
        const e2 = self.points[2].subtract(self.points[0]);
        const ray_cross_e2 = ray.direction.crossProduct(e2);

        const det = e1.dotProduct(ray_cross_e2);
        const epsilon: f32 = std.math.floatEps(f32);

        if (det > -epsilon and det < epsilon) {
            return hit;
        }

        const inv_det = 1.0 / det;
        const s = ray.position.subtract(self.points[0]);
        const u = inv_det * s.dotProduct(ray_cross_e2);

        if (u < 0 or u > 1) {
            return hit;
        }

        const s_cross_e1 = s.crossProduct(e1);
        const v = inv_det * ray.direction.dotProduct(s_cross_e1);

        if (v < 0 or u + v > 1) {
            return hit;
        }

        const t = inv_det * e2.dotProduct(s_cross_e1);
        if (t > epsilon) {
            hit.hit = true;
            hit.distance = t;
            hit.point = ray.position.add(ray.direction.scale(t));

            const normal = self.getNormal();
            if (ray.direction.dotProduct(normal) < 0) {
                hit.normal = normal;
            } else {
                hit.normal = normal.negate();
            }
        }

        return hit;
    }

    fn sameSide(p1: rl.Vector3, p2: rl.Vector3, v1: rl.Vector3, v2: rl.Vector3) bool {
        const edge = v2.subtract(v1);
        const n1 = edge.crossProduct(p1.subtract(v1));
        const n2 = edge.crossProduct(p2.subtract(v1));
        return n1.dotProduct(n2) >= 0;
    }

    //using same side collision
    pub fn rayIntersectTriangleSS(self: *const Triangle, ray: rl.Ray) rl.RayCollision {
        const normal = self.getNormal();
        const distance = self.getDistance();

        const plane = Plane.init(normal, distance);
        var hit = plane.rayIntersectPlane(ray);
        if (!hit.hit) {
            return hit;
        }

        const point = hit.point;
        hit.hit = sameSide(point, self.points[2], self.points[0], self.points[1]) and
            sameSide(point, self.points[0], self.points[1], self.points[2]) and
            sameSide(point, self.points[1], self.points[2], self.points[0]);

        if (ray.direction.dotProduct(hit.normal) > 0) {
            hit.normal = hit.normal.negate();
        }

        return hit;
    }

    //barycentric coordinates
    pub fn rayIntersectTriangleBT(self: *const Triangle, ray: rl.Ray) rl.RayCollision {
        const normal = self.getNormal();
        const distance = self.getDistance();
        const plane = Plane.init(normal, distance);

        var hit = plane.rayIntersectPlane(ray);

        if (!hit.hit) {
            return hit;
        }

        hit.hit = false;

        const point = hit.point;

        const v0 = self.points[1].subtract(self.points[0]);
        const v1 = self.points[2].subtract(self.points[0]);
        const v2 = point.subtract(self.points[0]);

        const dot_00 = v0.dotProduct(v0);
        const dot_01 = v0.dotProduct(v1);
        const dot_11 = v1.dotProduct(v1);

        const px = v0.dotProduct(v2);
        const py = v1.dotProduct(v2);

        const det = dot_00 * dot_11 - dot_01 * dot_01;
        const inv_det = 1 / det;

        const u = (px * dot_11 - dot_01 * py) * inv_det;
        const v = (dot_00 * py - px * dot_01) * inv_det;

        if (u >= 0 and v >= 0 and u + v < 1) {
            hit.hit = true;
            if (normal.dotProduct(ray.direction) > 0) {
                hit.normal = hit.normal.negate();
            }
        }

        return hit;
    }
};

pub const Sphere = struct {
    radius: f32,

    pub fn init(radius: f32) Sphere {
        return .{ .radius = radius };
    }

    pub fn getRadius(self: *const Sphere) f32 {
        return self.radius;
    }

    pub fn rayIntersectSphere(self: *const Sphere, ray: rl.Ray, position: rl.Vector3) rl.RayCollision {
        const dir = position.subtract(ray.position);
        const proj = dir.dotProduct(ray.direction);
        const p = ray.position.add(ray.direction.scale(proj));

        const dist2 = position.subtract(p).lengthSqr();
        const radius2 = self.radius * self.radius;

        var hit = rl.RayCollision{ .hit = false, .distance = 0.0, .point = rl.Vector3.zero(), .normal = rl.Vector3.zero() };

        if (dist2 <= radius2) {
            const l2 = p.subtract(position).lengthSqr();
            const d = std.math.sqrt(radius2 - l2);
            hit.distance = p.subtract(ray.position).length() - d;
            hit.hit = true;
            hit.point = ray.position.add(ray.direction.scale(hit.distance));
            hit.normal = hit.point.subtract(position).normalize();
        }

        return hit;
    }
};

pub const AABB = struct {
    half_extents: rl.Vector3,

    pub fn init(half_extents: rl.Vector3) AABB {
        return .{ .half_extents = half_extents };
    }

    pub fn getSize(self: *const AABB) rl.Vector3 {
        return self.half_extents.scale(2);
    }

    pub fn getMin(self: *const AABB, position: rl.Vector3) rl.Vector3 {
        return position.subtract(self.half_extents);
    }

    pub fn getMax(self: *const AABB, position: rl.Vector3) rl.Vector3 {
        return position.add(self.half_extents);
    }

    pub fn rayIntersectAABB(self: *const AABB, ray: rl.Ray, position: rl.Vector3) rl.RayCollision {
        const min = self.getMin(position);
        const max = self.getMax(position);

        const x_plane = if (ray.direction.x > 0) -min.x else max.x;
        const y_plane = if (ray.direction.y > 0) -min.y else max.y;
        const z_plane = if (ray.direction.z > 0) -min.z else max.z;

        const x_sign = std.math.sign(ray.direction.x);
        const y_sign = std.math.sign(ray.direction.y);
        const z_sign = std.math.sign(ray.direction.z);

        var hit = rl.RayCollision{ .hit = false, .distance = 0, .point = rl.Vector3.zero(), .normal = rl.Vector3.zero() };

        const x_normal = rl.Vector3.init(-x_sign, 0, 0);
        const y_normal = rl.Vector3.init(0, -y_sign, 0);
        const z_normal = rl.Vector3.init(0, 0, -z_sign);

        const plane_x = Plane.init(x_normal, x_plane);
        const plane_y = Plane.init(y_normal, y_plane);
        const plane_z = Plane.init(z_normal, z_plane);

        const hit_x = plane_x.rayIntersectPlane(ray);
        const hit_y = plane_y.rayIntersectPlane(ray);
        const hit_z = plane_z.rayIntersectPlane(ray);

        if (!hit_x.hit and !hit_y.hit and !hit_z.hit) {
            return hit;
        }

        const hit_x_dist = @abs(hit_x.distance);
        const hit_y_dist = @abs(hit_y.distance);
        const hit_z_dist = @abs(hit_z.distance);

        const hit_dist = @max(hit_x_dist, @max(hit_y_dist, hit_z_dist));

        if (hit_dist == hit_x_dist) {
            hit = hit_x;
        } else if (hit_dist == hit_y_dist) {
            hit = hit_y;
        } else if (hit_dist == hit_z_dist) {
            hit = hit_z;
        } else {
            unreachable;
        }

        const abs_x = @abs(position.x - hit.point.x) - 0.001;
        const abs_y = @abs(position.y - hit.point.y) - 0.001;
        const abs_z = @abs(position.z - hit.point.z) - 0.001;

        const half_size = self.half_extents;
        hit.hit = abs_x < half_size.x and abs_y < half_size.y and abs_z < half_size.z;

        return hit;
    }

    pub fn rayIntersectOBB(self: *const AABB, ray: rl.Ray, rotation: rl.Matrix, position: rl.Vector3) rl.RayCollision {
        const q = rl.Quaternion.fromMatrix(rotation);
        const q_inv = rl.Quaternion.fromMatrix(rotation.transpose());

        const ray_pos_local = ray.position.subtract(position).rotateByQuaternion(q_inv);
        const ray_dir_local = ray.direction.rotateByQuaternion(q_inv);

        const ray_local = .{ .position = ray_pos_local, .direction = ray_dir_local };

        var hit = AABB.init(self.half_extents).rayIntersectAABB(ray_local, rl.Vector3.zero());

        hit.point = hit.point.rotateByQuaternion(q).add(position);
        hit.normal = hit.normal.rotateByQuaternion(q);

        return hit;
    }
};

pub const Plane = struct {
    normal: rl.Vector3,
    distance: f32,

    pub fn init(normal: rl.Vector3, distance: f32) Plane {
        return .{ .normal = normal, .distance = distance };
    }

    pub fn rayIntersectPlane(self: *const Plane, ray: rl.Ray) rl.RayCollision {
        const numerator = ray.position.dotProduct(self.normal);
        const denominator = ray.direction.dotProduct(self.normal);

        const zero = rl.Vector3.zero();
        var hit = rl.RayCollision{ .hit = false, .point = zero, .normal = zero, .distance = 0 };

        if (denominator == 0) {
            return hit;
        }

        const t = (-numerator + self.distance) / denominator;

        if (t < 0) {
            return hit;
        }

        hit.hit = true;
        hit.distance = t;
        hit.point = ray.position.add(ray.direction.scale(t));
        hit.normal = self.normal;

        return hit;
    }
};
