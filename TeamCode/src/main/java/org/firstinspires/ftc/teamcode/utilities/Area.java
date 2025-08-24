package org.firstinspires.ftc.teamcode.utilities;

public class Area {

    /**
     * The size of the area in the x direction.
     */
    private final double width;
    /**
     * The size of the area in the y direction.
     */
    private final double height;

    private final Pose topLeft;
    private final Pose topRight;
    private final Pose bottomLeft;
    private final Pose bottomRight;
    private final Pose center;

    public Area(Pose topLeft, double width, double height){
        this(topLeft, topLeft.plusX(width).plusY(-height));
    }

    public Area(Pose topLeft, Pose bottomRight){
        this.width = bottomRight.x() - topLeft.x();
        this.height = topLeft.y() - bottomRight.y();
        this.topLeft = topLeft;
        this.topRight = topLeft.plusX(width);
        this.bottomLeft = topLeft.plusY(-height);
        this.bottomRight = bottomRight;
        this.center = topLeft.plusX(width/2.0).plusY(-height/2.0);
    }

    public Pose getTopLeft(){
        return topLeft;
    }

    public Pose getTopRight() {
        return topRight;
    }

    public Pose getBottomLeft() {
        return bottomLeft;
    }

    public Pose getBottomRight() {
        return bottomRight;
    }

    public Pose getCenter() {
        return center;
    }

    /**
     * @return the smallest x value within the area.
     */
    public double getMinX(){
        return bottomLeft.x();
    }

    /**
     * @return the largest x value within the area.
     */
    public double getMaxX(){
        return topRight.x();
    }

    /**
     * @return the smallest y value within the area.
     */
    public double getMinY(){
        return bottomLeft.y();
    }

    /**
     * @return the largest y value within the area.
     */
    public double getMaxY(){
        return topRight.y();
    }

    /**
     * Get the size of the area in the y direction.
     */
    public double getHeight() {
        return height;
    }

    /**
     * Get the size of the area in the x direction.
     */
    public double getWidth() {
        return width;
    }

    public boolean containsPose(Pose pose){
        return ((pose.x() >= getMinX() && pose.x() <= getMaxX()) && (pose.y() >= getMinY() && pose.y() <= getMaxY()));
    }
}
