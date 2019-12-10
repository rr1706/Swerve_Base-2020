package frc.robot.utilities;

public class Pair<T,  K> {
    private T first;
    private K second;

    public Pair(T first, K second) {
        set(first, second);
    }

    public void set(T  first, K second) {
        this.first = first;
        this.second = second;
    }

    public T getFirst() {
        return first;
    }

    public  K getSecond() {
        return  second;
    }

}
