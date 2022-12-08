public class Example1 {
  private int x = 1;
  private int y = 2;

  public void A() {
    if (x < y) {
      B();
    } else {
      if (x < y) {
        B();
      } else if (2 * x > 3) {
        B();
      } else {
        B();
      }
    }
  }

  public int B() {
    if (x < y) {
      B();
    } else {
      if (x < y) {
        B();
      } else {
        B();
      }
    }
    return 0;
  }
}
