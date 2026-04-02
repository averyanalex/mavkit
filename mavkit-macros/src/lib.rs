use proc_macro::TokenStream;

/// Generates wire conversion functions for a mission command struct.
///
/// See the crate-level documentation for usage details.
#[proc_macro_attribute]
pub fn mavkit_command(_attr: TokenStream, item: TokenStream) -> TokenStream {
    // Placeholder — returns input unchanged.
    // Full implementation in Task 3.2.
    item
}
