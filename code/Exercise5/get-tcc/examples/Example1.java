public class Example1 {
  private int x = 1;
  private int y = 2;

  public void A() {
    B();
  }

  public int B() {
    x += 1;
    return x;
  }

  public void C() {
    System.out.println(y--);
  }

  public void D() {
    y -= 3;
    E();
  }

  public void E() {
    System.out.println("Hello");
  }
}
