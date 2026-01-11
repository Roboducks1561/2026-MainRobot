package frc.robot.util;

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2 add(Vector2 other) {
        return new Vector2(this.x + other.x, this.y + other.y);
    }

    public Vector2 subtract(Vector2 other) {
        return new Vector2(this.x - other.x, this.y - other.y);
    }

    public Vector2 multiply(double scalar) {
        return new Vector2(this.x * scalar, this.y * scalar);
    }

    public double dot(Vector2 other) {
        return this.x * other.x + this.y * other.y;
    }

    public double cross(Vector2 other) {
        return this.x * other.y - this.y * other.x;
    }

    public double getMagnitude(){
        return Math.sqrt(x * x + y * y);
    }

    public Vector2 normalize() {
        double magnitude = Math.sqrt(x * x + y * y);
        return new Vector2(x / magnitude, y / magnitude);
    }

    public Vector2 perpendicular() {
        return new Vector2(-this.y, this.x); // 90-degree rotation
    }

    public Vector2 rotate(double angleRadians) {
        double cosTheta = Math.cos(angleRadians);
        double sinTheta = Math.sin(angleRadians);
        double newX = x * cosTheta - y * sinTheta;
        double newY = x * sinTheta + y * cosTheta;
        return new Vector2(newX, newY);
    }
}