package Canvas.Util;

import java.util.function.Supplier;

@FunctionalInterface
public interface ParamSupplier<O, I, I2> {

    /**
     * Performs this operation on the given argument.
     *
     * @param i1 input 1
     * @param i2 input 2
     */
    O accept(I i1, I2 i2);
}

