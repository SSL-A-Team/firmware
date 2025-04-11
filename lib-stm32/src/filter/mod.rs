pub trait Filter<T> {
    fn add_sample(T: sample) -> T;

    fn get_value() -> T;
}